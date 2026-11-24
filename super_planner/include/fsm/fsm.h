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

#include <queue>
#include <memory>
#include <mutex>
#include <atomic>
#include <fstream>
#include <fmt/color.h>
#include <cereal/archives/binary_file_handler.hpp>
#include <fsm/config.hpp>
#include <super_core/super_planner.h>


#ifndef LOG_FILE_DIR
#define LOG_FILE_DIR(name) (string(string(ROOT_DIR) + "log/"+name))
#endif

namespace fsm {
    class Fsm {
    protected:
        bool stop{false};


        vector<string> log_time_str{
                "TIME_STAMPE", "EPX_TRAJ_FRONTEND",
                "EXP_TRAJ_OPT", "GENERATE_EXP_TRAJ",
                "BACK_TRAJ_FRONTEND", "BACK_TRAJ_OPT",
                "GENERATE_BACK_TRAJ", "TOTAL_REPLAN", "VISUALIZATION"
        };
        Config cfg_;
        // map, checker, planner
        super_planner::SuperPlanner::Ptr planner_ptr_;
        ros_interface::RosInterface::Ptr ros_ptr_;

        std::ofstream write_time_;
        vector<double> log_module_time;
        double yaw_{0}, yaw_dot_{0};

        // Written by the FSM timer (callMainFsmOnce) under robot_state_mutex_; that thread also
        // reads it without the lock. Other threads read the position through robotPosition(), or
        // copy the whole state under the lock (the hover guard in FsmRos1::pubCmdTimerCallback).
        rog_map::RobotState robot_state_;
        std::mutex robot_state_mutex_;

        Vec3f robotPosition() {
            std::lock_guard<std::mutex> lck(robot_state_mutex_);
            return robot_state_.p;
        }

        // params
        bool started_{false}, plan_from_rest_{false};

        struct GoalInfo {
            bool new_goal;
            Vec3f goal_p;
            double goal_yaw;
            Vec3f click_p;  // last accepted goal as received (before snapping), for the regoal_dist gate
        } gi_;

        // Goal race: the goal callback and the FSM/replan timers run on different threads
        // (ROS1 AsyncSpinner(0), ROS2 separate callback groups). goal_mutex_ guards gi_,
        // finish_plan and goal_seq_. Planning works on a locked copy of the goal; results
        // (snapped goal, new_goal = false, finish_plan) are written back only if goal_seq_ is
        // unchanged, so a goal accepted meanwhile is never overwritten by the old one. Without
        // this, the regoal_dist gate would ignore the follow-up messages that used to repair it.
        std::mutex goal_mutex_;
        uint64_t goal_seq_{0};

        Eigen::Vector3d auto_pilot_vel_w_;

        // execution states
        enum MACHINE_STATE {
            INIT = 0,
            WAIT_GOAL,
            YAWING,
            GENERATE_TRAJ,
            FOLLOW_TRAJ,
            EMER_STOP,
            // Stopping along the committed trajectory (see startBrake); ends in WAIT_GOAL.
            BRAKE
        };

        vector<string> MACHINE_STATE_STR{
                "INIT",
                "WAIT_GOAL",
                "YAWING",
                "GENERATE_TRAJ",
                "FOLLOW_TRAJ", "EMER_STOP", "BRAKE"
        };


        // Atomic: read by the goal callback; read and written by the cmd, FSM and replan timers.
        // Check-then-act sequences on it are not atomic (as upstream); see FsmRos1::pubCmdTimerCallback.
        std::atomic<MACHINE_STATE> machine_state_{INIT};


    public:
        Fsm() = default;
        ~Fsm();

        void updateROGMap(const rog_map::PointCloud &cloud, const super_utils::Pose &pose) {
            planner_ptr_->updateROGMap(cloud, pose);
        }

        void callPlanOnce(const Vec3f &goal) {
            TimeConsuming tc("Call replan once time", true);
            fmt::print(" -- [Fsm] Call plan once, cur state {}.\n", MACHINE_STATE_STR[machine_state_.load()]);
            // check current state;
            Quatf q(NAN, NAN, NAN, NAN);
            setGoalPosiAndYaw(goal, q);

            callMainFsmOnce();

            if (machine_state_ == FOLLOW_TRAJ) {
                callReplanOnce();
            }

            // save on log
            replan_logs_.push_back(planner_ptr_->getLatestReplanLog());
            fmt::print(fmt::fg(fmt::color::green), " -- Replan ID: {}, ret code: {}\n",
                       replan_logs_.size() - 1, replan_logs_.back().getRetCode());
        }

        Eigen::Quaterniond eulerToQuaternion(double roll, double pitch, double yaw) {
            double half_roll = roll * 0.5;
            double half_pitch = pitch * 0.5;
            double half_yaw = yaw * 0.5;

            double sin_r = std::sin(half_roll);
            double cos_r = std::cos(half_roll);
            double sin_p = std::sin(half_pitch);
            double cos_p = std::cos(half_pitch);
            double sin_y = std::sin(half_yaw);
            double cos_y = std::cos(half_yaw);

            // 计算四元数分量
            Eigen::Quaterniond q;
            q.w() = cos_r * cos_p * cos_y + sin_r * sin_p * sin_y;
            q.x() = sin_r * cos_p * cos_y - cos_r * sin_p * sin_y;
            q.y() = cos_r * sin_p * cos_y + sin_r * cos_p * sin_y;
            q.z() = cos_r * cos_p * sin_y - sin_r * sin_p * cos_y;

            return q;
        }

    protected:
        vector<LogOneReplan> replan_logs_;
        /* Callback functions */
        bool finish_plan = false;
        double system_start_time_;

        bool traj_finish_{false};

        // True once the committed trajectory has been flown to its end in FOLLOW_TRAJ (set by the
        // command output when it sees traj_finish, which also latches the hover point), so the
        // drone rests at that point. It survives GENERATE_TRAJ -> WAIT_GOAL (upstream's path when
        // the end is 0.1 m or more from the goal, and after a new goal turns out invalid; neither
        // commits anything) and is cleared on entering FOLLOW_TRAJ (a new trajectory), EMER_STOP
        // or INIT (ChangeState). A PlanFromRest failure stays in GENERATE_TRAJ and retries, with
        // nothing published. Gates the yaw_mode 3 hover command in WAIT_GOAL
        // (FsmRos1::pubCmdTimerCallback).
        std::atomic<bool> hold_cmd_valid_{false};

        // Brake (FOLLOW_TRAJ -> BRAKE -> WAIT_GOAL): the committed trajectory is replaced by a stop
        // along its own path (SuperPlanner::prepareBrake), sent by the cmd output; replans stop
        // (callReplanOnce runs only in FOLLOW_TRAJ). Trigger: an accepted hold request.
        // brake_mutex_ serialises startBrake (goal callback and replan timer).
        std::mutex brake_mutex_;
        // goal_seq_ when the brake started, and whether a hold request started it (see finishBrake).
        std::atomic<uint64_t> brake_seq_{0};
        std::atomic<bool> brake_for_hold_{false};

        /// True when the cmd output sends brake commands in BRAKE. Without it no brake starts and
        /// the upstream behaviour is kept (ROS2).
        virtual bool brakeSupported() const {
            return false;
        }

        /// Start a brake if the state is FOLLOW_TRAJ and a trajectory is still being flown.
        /// trigger names it in the log; seq is the goal_seq_ it was decided for; hold is true for
        /// a hold request. Returns false, changing nothing, otherwise: in another state the brake
        /// is not needed (BRAKE: already braking) or not possible (GENERATE_TRAJ: no trajectory is
        /// flown, nothing is published), and after the trajectory end the drone is at rest there.
        bool startBrake(const char *trigger, const uint64_t &seq, const bool &hold);

        /// Called by the cmd output when the brake profile has ended and the stop point is latched
        /// as the hover point: BRAKE -> WAIT_GOAL with the hover command valid. After a hold
        /// request, if no goal came in since, the goal becomes the stop point (new_goal false), so
        /// the drone hovers there instead of flying back to the hold point it braked past. Else
        /// the newer goal (or, after the replan trigger, the goal that failed) is planned from rest
        /// in GENERATE_TRAJ; until a trajectory is committed nothing is published there and the
        /// controller keeps the last brake command: the stop point at zero velocity.
        void finishBrake();

        /// ChangeState only if the state is still expected (atomic compare-and-swap). A timer that
        /// decided on a stale state then leaves a brake started meanwhile in place.
        bool changeStateFrom(const string &call_func, MACHINE_STATE expected, const MACHINE_STATE &new_state);

        void WriteTimeToLog();

        void callReplanOnce();

        void callMainFsmOnce();

        bool closeToGoal(const double &thresh_dis);

        void setGoalPosiAndYaw(const Vec3f &p, const Quatf &q);

        void ChangeState(const string &call_func, const MACHINE_STATE &new_state);

        virtual void publishPolyTraj() = 0;

        virtual void publishCurPoseToPath() = 0;

        virtual void resetVisualizedPath() = 0;
    };
}
