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
        // Hover point latched by latchHoldPoint() when a trajectory ends (cmd timer thread only).
        Vec3f hold_p_{0, 0, 0};
        double hold_yaw_{NAN};

        void applyLookAtYaw(const Vec3f &cmd_p, const double &cur_WT, const bool &hold_fallback,
                            double &yaw, double &yaw_dot);

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

        const Vec3f &getHoldPoint() const {
            return hold_p_;
        }

        /// yaw_mode 3 hover command: the latched hover point (never the live trajectory) with
        /// zero velocity, acceleration and jerk, and the look-at yaw. Without a fresh target the
        /// last commanded yaw is held (yaw_dot 0).
        void getHoldCommand(StatePVAJ &pvaj, double &yaw, double &yaw_dot);

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
