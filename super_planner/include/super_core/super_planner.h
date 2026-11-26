/**
* This file is part of SUPER
*
* Copyright 2025 Yunfan REN, MaRS Lab, University of Hong Kong, <mars.hku.hk>
* Developed by Yunfan REN <renyf at connect dot hku dot hk>
* for more information see <https://github.com/hku-mars/SUPER>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* SUPER is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* SUPER is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with SUPER. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include <iostream>
#include <fstream>
#include "Eigen/Eigen"


#include <super_core/config.hpp>
#include <ros_interface/ros1/ros1_interface.hpp>
#include <data_structure/base/trajectory.h>

#include <data_structure/base/polytope.h>


#include "traj_opt/exp_traj_optimizer_s4.h"
#include "traj_opt/backup_traj_optimizer_s4.h"
#include "path_search/astar.h"
#include "rog_map/rog_map.h"
#include "super_core/corridor_generator.h"
#include "super_core/fov_checker.h"

#include "traj_opt/yaw_traj_opt.h"
#include "super_core/super_ret_code.hpp"
#include "utils/header/fmt_eigen.hpp"

#include <super_core/log_utils.hpp>
#include <data_structure/exp_traj.h>
#include <data_structure/cmd_traj.h>
#include <data_structure/backup_traj.h>


namespace super_planner {
    using namespace color_text;
    using namespace geometry_utils;

    class SuperPlanner {
        LogOneReplan latest_replan;
        super_planner::Config cfg_;
        rog_map::ROGMapROS::Ptr map_ptr_;
        CorridorGenerator::Ptr cg_ptr_;
        path_search::Astar::Ptr astar_ptr_;
        ros_interface::RosInterface::Ptr ros_ptr_;
        Vec3f shifted_sfc_start_pt_;

        traj_opt::ExpTrajOpt::Ptr exp_traj_opt_;
        traj_opt::BackupTrajOpt::Ptr back_traj_opt_;
        traj_opt::YawTrajOpt::Ptr yaw_traj_opt_;

        CIRI::Ptr ciri_;

        super_utils::RobotState robot_state_;

        std::mutex drone_state_mutex_;
        std::mutex replan_lock_;

        Vec3f local_start_p_;

        bool robot_on_backup_traj_{false};
        // use negative value to indicate the traj is not available
        double on_backup_start_WT{-1}, on_backup_end_WT{-1};

        double planner_process_start_WT_;

        struct GoalInfo {
            Vec3f goal_p{0, 0, 0};
            double goal_yaw{0};
            bool new_goal{true};
            bool goal_valid{true};
        } gi_;

        FOVChecker::Ptr fov_checker_;

        CmdTraj cmd_traj_info_;
        ExpTraj last_exp_traj_info_;

        vector<double> time_consuming_;

        /* yaw_mode 3 (look at a point) */
        // Near the target horizontally the bearing is undefined (target above/below the drone)
        // or jumps with a few cm of noise: the commanded yaw is held. Hysteresis: the hold starts
        // below the enter distance [m] and ends above the leave distance [m].
        static constexpr double kLookAtHoldEnterDis = 0.1;
        static constexpr double kLookAtHoldLeaveDis = 0.2;
        // Cap on the time step [s] of the yaw rate limiter, so a gap between commands does not
        // turn into one large yaw step.
        static constexpr double kLookAtMaxDt = 0.05;
        // A target stamped more than this [s] in the future is rejected on receipt.
        static constexpr double kYawTargetMaxFuture = 0.5;
        // A stored target whose (receive-clamped) stamp is ahead of now by more than this [s] was
        // received before the clock jumped back (sim or bag clock reset) and is dropped. The slack
        // only absorbs the callback storing a slightly later "now" than the command thread read.
        static constexpr double kYawTargetClockSlack = 0.05;
        std::mutex yaw_target_mutex_;
        Vec3f yaw_target_p_{0, 0, 0};
        double yaw_target_WT_{-1};  // < 0: no target
        double look_at_yaw_{NAN};   // last commanded yaw, continuous (not wrapped); NAN until the first command
        double look_at_last_WT_{-1};
        bool look_at_hold_{false};  // inside the hold radius (hysteresis state)
        // Hover point latched by latchHoldPoint() when a trajectory ends, by latchBrakeStopPoint()
        // when a brake ends (both from the cmd output), and by latchHoldAt() on a hold request
        // (hold topic callback). Read by the cmd output. Guarded by hold_mutex_.
        mutable std::mutex hold_mutex_;
        Vec3f hold_p_{0, 0, 0};
        double hold_yaw_{NAN};

        void applyLookAtYaw(const Vec3f &cmd_p, const double &cur_WT, const bool &hold_fallback,
                            double &yaw, double &yaw_dot);

        /* Brake: stop along the committed trajectory (prepareBrake / getBrakeCommand) */
        // Integration step [s] of the brake profile; one sample is stored per step.
        static constexpr double kBrakeDt = 0.001;
        // Below this speed [m/s] the brake ends: the speed is set to 0 and the drone hovers. Near
        // the stop the deceleration is held at sqrt(2 max_jerk v), so the command's acceleration
        // steps to 0 by at most sqrt(2 max_jerk kBrakeStopSpeed) (0.49 m/s^2 at max_jerk 120).
        static constexpr double kBrakeStopSpeed = 0.001;
        // Cap on the profile length [s]. It is never reached with max_acc and max_jerk > 0 (5 m/s
        // at 5 m/s^2 stops in about 1 s); if it were, the profile ends there with speed 0, so the
        // command still ends in a hover, with a velocity step.
        static constexpr double kBrakeMaxDuration = 10.0;
        // The brake runs the committed trajectory on a slowed clock: the command at time t is the
        // trajectory state at path time s(t), with s' = ds/dt falling from 1 to 0 (stored per step:
        // s, s' and s'' = u). Written by prepareBrake, read by getBrakeCommand, under brake_mutex_.
        struct BrakeProfile {
            Trajectory pos_traj, yaw_traj;
            double start_WT{0};
            vector<double> s, sd, u;
            Vec3f stop_p{0, 0, 0};
            double stop_yaw{NAN};
        };
        std::mutex brake_mutex_;
        BrakeProfile brake_;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        explicit SuperPlanner(const std::string &cfg_path,
                              const ros_interface::RosInterface::Ptr &ros_ptr,
                              const rog_map::ROGMapROS::Ptr &map_ptr);

        ~SuperPlanner() = default;

        void lockCommittedTraj() {
            cmd_traj_info_.lock();
        }

        void unlockCommittedTraj() {
            cmd_traj_info_.unlock();
        }

        bool goalValid() const {
            return gi_.goal_valid;
        }

        bool yawToPointEnabled() const {
            return cfg_.yaw_mode == Config::YAW_TO_POINT;
        }

        /// yaw_mode 3: set the world-frame point the commanded yaw looks at. stamp_WT is the
        /// target's time [s] (same clock as RosInterface::getSimTime). A stamp up to
        /// kYawTargetMaxFuture in the future is clamped to now; later ones, and non-finite
        /// points, are ignored.
        void setYawTarget(const Vec3f &p, const double &stamp_WT);

        /// yaw_mode 3: latch the final point (and yaw) of the committed trajectory as the hover
        /// point. Call when the trajectory has just been flown to its end.
        void latchHoldPoint();

        Vec3f getHoldPoint() const {
            std::lock_guard<std::mutex> lck(hold_mutex_);
            return hold_p_;
        }

        /// Latch p (yaw: the yaw to hold, NaN for the look-at yaw only) as the hover point: a
        /// hold request while hovering or idle, or while planning from rest. Thread-safe.
        void latchHoldAt(const Vec3f &p, const double &yaw) {
            std::lock_guard<std::mutex> lck(hold_mutex_);
            hold_p_ = p;
            hold_yaw_ = yaw;
        }

        /// yaw_mode 3 hover command: the latched hover point (never the live trajectory) with
        /// zero velocity, acceleration and jerk, and the look-at yaw. Without a fresh target the
        /// last commanded yaw is held (yaw_dot 0).
        void getHoldCommand(StatePVAJ &pvaj, double &yaw, double &yaw_dot);

        /// Plan a brake from now along the committed trajectory: the same path, slowed until the
        /// speed is 0, with the commanded acceleration within traj_opt max_acc and its rate within
        /// max_jerk (exp_traj limits). The path is the already checked trajectory, so the brake
        /// keeps its clearance; it stops at or before the trajectory end (where the trajectory
        /// itself is at rest). Returns false, planning nothing, when no trajectory is committed or
        /// it has already been flown to its end (the drone is then at rest at its end). On
        /// success, speed is the speed at the start [m/s], and stop_p, stop_dist [m] and
        /// stop_time [s] describe where and when the drone stops. For the log, the trajectory it
        /// replaces: traj_end_p is where it would have ended (at rest), traj_left [s] how much of
        /// it was still to fly, and backup_in [s] when its backup (stop) segment starts from now
        /// (< 0: already on it; NaN: it has none). Thread-safe.
        bool prepareBrake(const double &now_WT, double &speed, Vec3f &stop_p, double &stop_dist,
                          double &stop_time, Vec3f &traj_end_p, double &traj_left, double &backup_in);

        /// Brake command at the current time from the last prepareBrake. finished is true once
        /// the profile has ended: the command is then the stop point with zero velocity,
        /// acceleration and jerk. With yaw_mode 3 the yaw comes from look-at as in
        /// getOneCommandFromTraj. Thread-safe.
        void getBrakeCommand(StatePVAJ &pvaj, double &yaw, double &yaw_dot, bool &finished);

        /// Latch the brake's stop point as the hover point (see latchHoldPoint). Call from the cmd
        /// output when getBrakeCommand reports finished.
        void latchBrakeStopPoint();

        typedef std::shared_ptr<SuperPlanner> Ptr;

        void getOneHeartbeatTime(double &start_WT_pos, bool &traj_finish);

        Trajectory getCommittedPositionTrajectory();

        Trajectory getCommittedYawTrajectory();

        void getOneCommandFromTraj(StatePVAJ &pvaj,
                                   double &yaw,
                                   double &yaw_dot,
                                   bool &on_backup_traj,
                                   bool &traj_finish);

        void getModuleTimeConsuming(vector<double> &time);

        /* Tow type of replan strategy */
        RET_CODE PlanFromRest(const Vec3f &goal_p,
                              const double &goal_yaw,
                              const bool &new_goal);

        RET_CODE
        ReplanOnce(const Vec3f &goal_p,
                   const double &goal_yaw,
                   const bool &new_goal);

    private:
        RET_CODE generateExpTraj(ExpTraj &last_exp_traj_info,
                                 ExpTraj &out_exp_traj_info);

        /* For Backup traj generation */
        RET_CODE generateBackupTrajectory(ExpTraj &ref_exp_traj, BackupTraj &back_traj_info);

        int getNearestFurtherGoalPoint(const vec_E<Vec3f> &goals, const Vec3f &start_pt);

        bool PathSearch(const Vec3f &start_pt, const Vec3f &goal,
                        const double &searching_horizon,
                        vec_Vec3f &path);


    public:
        void getRobotState(rog_map::RobotState &out);

        bool isEasyGoal(const Vec3f &goal_position);

        rog_map::ROGMapROS::Ptr &getMap() {
            return map_ptr_;
        }

        double ft{0}, bt{0};
        int ft_cnt{0}, bt_cnt{0};

        double getFrontendTime() {
            if (ft_cnt == 0) return -1;
            double ave_t = ft / ft_cnt;
            ft = 0;
            ft_cnt = 0;
            return ave_t;
        }

        double getBackendTime() {
            if (bt_cnt == 0) return -1;
            double ave_t = bt / bt_cnt;
            bt = 0;
            bt_cnt = 0;
            return ave_t;
        }

        void updateROGMap(const rog_map::PointCloud &cloud, const super_utils::Pose &pose) const {
            map_ptr_->updateMap(cloud, pose);
        }

        LogOneReplan getLatestReplanLog() {
            latest_replan.setSfcPc(cg_ptr_->getLatestCloud());
            latest_replan.setComptT(time_consuming_);
            return latest_replan;
        }
    };
}
