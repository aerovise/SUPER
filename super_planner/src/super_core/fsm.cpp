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

#include <fsm/fsm.h>
#include <memory>

using namespace super_utils;

namespace fsm {
    Fsm::~Fsm() {
        write_time_.close();
    }

    void Fsm::WriteTimeToLog() {
        write_time_ << (ros_ptr_->getSimTime() - system_start_time_) << ", ";
        for (long unsigned int i = 0; i < log_module_time.size(); i++) {
            write_time_ << log_module_time[i];
            if (i != log_module_time.size() - 1) {
                write_time_ << ", ";
            }
        }
        write_time_ << endl;
    }

    void Fsm::callReplanOnce() {
        if (stop) {
            return;
        }

        if (machine_state_ != FOLLOW_TRAJ) {
            return;
        }

        Vec3f goal_p;
        double goal_yaw;
        bool new_goal;
        uint64_t seq;
        {
            std::lock_guard<std::mutex> lck(goal_mutex_);
            if (finish_plan) {
                return;
            }
            goal_p = gi_.goal_p;
            goal_yaw = gi_.goal_yaw;
            new_goal = gi_.new_goal;
            seq = goal_seq_;
        }

        if (plan_from_rest_) {
            plan_from_rest_ = false;
            // A new trajectory from rest: failures of earlier ones do not count toward the brake.
            replan_fail_since_ = -1;
            return;
        }

        // Snap the local copy (upstream snapped gi_.goal_p in place, racing the goal callback).
        planner_ptr_->getMap()->getNearestInfCellNot(GridType::OCCUPIED, goal_p, goal_p, 3.0);
        {
            std::lock_guard<std::mutex> lck(goal_mutex_);
            if (seq == goal_seq_) {
                gi_.goal_p = goal_p;
            }
        }

        TimeConsuming replan_once_time("replan_once_time", false);

        RET_CODE ret_code = planner_ptr_->ReplanOnce(goal_p, goal_yaw, new_goal);
        if (ret_code == FAILED) {
//            cout << YELLOW << " -- [Fsm] ReplanOnce failed." << RESET << endl;
        } else { cout << GREEN << " -- [Fsm] ReplanOnce succeed." << RESET << endl; }

        // The state changes only from FOLLOW_TRAJ: a brake started while ReplanOnce ran stays in
        // place, and whatever ReplanOnce committed is not flown (the cmd output sends the brake,
        // which has its own copy of the trajectory).
        if (ret_code == EMER) {
            changeStateFrom("ReplanTimerCallback", FOLLOW_TRAJ, EMER_STOP);
        } else if (ret_code == NEW_TRAJ) {
            changeStateFrom("ReplanTimerCallback", FOLLOW_TRAJ, GENERATE_TRAJ);
        } else if (ret_code == SUCCESS || ret_code == FINISH) {
            committed_seq_ = seq;
            replan_fail_since_ = -1;
            {
                std::lock_guard<std::mutex> lck(goal_mutex_);
                if (seq == goal_seq_) {
                    gi_.new_goal = false;
                }
            }
            if (machine_state_ == FOLLOW_TRAJ) {
                publishPolyTraj();
            }
        } else if (ret_code == FAILED) {
            // Replan-failure brake. Upstream keeps flying the committed trajectory while replans
            // fail ("force return"). That is kept while the goal is the one the trajectory was
            // planned for: the trajectory still leads there and ends in its backup stop in known
            // free space. Once the goal has changed (e.g. the follower asks to retreat or hold),
            // the trajectory flies toward a place nobody asks for any more, at speed. When the
            // failures with a changed goal have lasted longer than brake_after_replan_fail, the
            // drone brakes along the trajectory and hovers at the stop point (startBrake), then
            // plans from rest toward the latest goal (finishBrake). If no brake is possible
            // (trajectory already flown to its end), the drone is at rest there.
            if (seq == committed_seq_) {
                replan_fail_since_ = -1;
            } else {
                const double now = ros_ptr_->getSimTime();
                if (replan_fail_since_ < 0) {
                    replan_fail_since_ = now;
                }
                const double failing_for = now - replan_fail_since_;
                if (failing_for > cfg_.brake_after_replan_fail) {
                    const std::string trigger = fmt::format(
                            "replan_fail: replans failing for {:.2f} s with a new goal", failing_for);
                    startBrake(trigger.c_str(), seq, false);
                }
            }
        }

        planner_ptr_->getModuleTimeConsuming(log_module_time);
        log_module_time[log_module_time.size() - 2] = replan_once_time.stop();
        // save on log
        replan_logs_.push_back(planner_ptr_->getLatestReplanLog());
        WriteTimeToLog();
    }

    void Fsm::callMainFsmOnce() {
        if (stop) {
            return;
        }
        static double fsm_start_time = ros_ptr_->getSimTime();
        double cur_t = (ros_ptr_->getSimTime() - fsm_start_time);
        static double last_print_t = 0.0;
        {
            rog_map::RobotState rs;
            planner_ptr_->getRobotState(rs);
            std::lock_guard<std::mutex> lck(robot_state_mutex_);
            robot_state_ = rs;
        }


        if (cur_t - last_print_t > 1.0) {
            last_print_t = cur_t;
            if ((!robot_state_.rcv || (ros_ptr_->getSimTime() - robot_state_.rcv_time) > 0.1)) {
                cout << YELLOW << " -- [Fsm] No odom." << RESET << endl;
                return;
            }
            if (!started_) {
                cout << YELLOW << " -- [Fsm] Wait for goal." << RESET << endl;
            }
            cout << std::fixed << std::setprecision(3);
            cout << GREEN << " -- [Fsm " << cur_t << "] Current state: " << MACHINE_STATE_STR[machine_state_.load()]
                 << RESET << endl;
        }

        switch (machine_state_) {
            case INIT: {
                if (!started_) {
                    return;
                }
                if ((!robot_state_.rcv || (ros_ptr_->getSimTime() - robot_state_.rcv_time) > 0.1)) {
                    cout << YELLOW << " -- [Fsm] No odom." << RESET << endl;
                }
                ChangeState("MainFsmCallback", WAIT_GOAL);
                break;
            }
            case WAIT_GOAL: {
                bool new_goal;
                {
                    std::lock_guard<std::mutex> lck(goal_mutex_);
                    new_goal = gi_.new_goal;
                }
                if (!new_goal) {
                    return;
                } else {
                    ChangeState("MainFsmCallback", GENERATE_TRAJ);
                }
                resetVisualizedPath();
                break;
            }
            case GENERATE_TRAJ: {
                Vec3f goal_p;
                double goal_yaw;
                bool new_goal;
                uint64_t seq;
                {
                    std::lock_guard<std::mutex> lck(goal_mutex_);
                    goal_p = gi_.goal_p;
                    goal_yaw = gi_.goal_yaw;
                    new_goal = gi_.new_goal;
                    seq = goal_seq_;
                }
                if ((robot_state_.p - goal_p).norm() < 0.1) {
                    std::lock_guard<std::mutex> lck(goal_mutex_);
                    if (seq == goal_seq_) {
                        ChangeState("MainFsmCallback", WAIT_GOAL);
                        gi_.new_goal = false;
                        finish_plan = true;
                    }
                    return;
                }
                int retcode = planner_ptr_->PlanFromRest(goal_p, goal_yaw, new_goal);
                if (!planner_ptr_->goalValid()) {
                    cout << YELLOW << " -- [Fsm] Goal is invalid, skip this goal." << RESET << endl;
                    ChangeState("MainFsmCallback", WAIT_GOAL);
                    return;
                }
                if (retcode == SUCCESS || retcode == FINISH) {
                    plan_from_rest_ = true;
                    committed_seq_ = seq;
                    {
                        std::lock_guard<std::mutex> lck(goal_mutex_);
                        if (seq == goal_seq_) {
                            gi_.new_goal = false;
                            finish_plan = retcode == FINISH;
                        }
                    }

                    publishPolyTraj();

                    ChangeState("MainFsmCallback", FOLLOW_TRAJ);
                } else {
                    cout << YELLOW << " -- [Fsm] PlanFromRest failed, try replan." << RESET << endl;
                    // ros::Duration(0.1).sleep();
                }
                replan_logs_.push_back(planner_ptr_->getLatestReplanLog());
                break;
            }
            case FOLLOW_TRAJ: {
                publishCurPoseToPath();
                break;
            }
            case EMER_STOP: {
                ChangeState("MainFsmCallback", WAIT_GOAL);
                break;
            }
            default:
                break;
        }
    }

    bool Fsm::closeToGoal(const double &thresh_dis) {
        /// The close to goal should consider the the local shift
        /// All goal should be in the known free on inf map.
        /// The intermedia points should be in free space.
        const Vec3f robot_p = robotPosition();  // before goal_mutex_: the two locks never nest
        std::lock_guard<std::mutex> lck(goal_mutex_);
        double dis = (robot_p - gi_.goal_p).norm();
        return dis < thresh_dis;
    }

    void Fsm::setGoalPosiAndYaw(const Vec3f &p, const Quatf &q) {

        auto click_point = p;
        if (cfg_.click_height > -5) {
            click_point.z() = cfg_.click_height;
        }

        // Robot position snapshot (the FSM timer updates robot_state_ concurrently).
        const Vec3f robot_p = robotPosition();

        // Snap into a local first: gi_.goal_p only changes once the goal is accepted below.
        Vec3f goal_p;
        if (!planner_ptr_->getMap()->getNearestInfCellNot(GridType::OCCUPIED, click_point, goal_p, 3.0)) {
            fmt::print(fg(fmt::color::indian_red), "Goal is deeply occupied, skip this goal.\n");
            return;
        }

        const MACHINE_STATE state = machine_state_;
        const bool flying = state == GENERATE_TRAJ || state == FOLLOW_TRAJ || state == BRAKE;

        // At rest, a goal within 0.1 m of the robot is skipped (upstream behaviour). While flying,
        // it is a hold request ("stop and hover here"): it bypasses the regoal_dist gate below and
        // always replaces the current goal; the planner brings the drone to rest at it and the FSM
        // ends in WAIT_GOAL. A follower should latch the hold point: re-sending the live drone
        // position while braking makes every message a new hold goal and forces a replan each time.
        const bool near_robot = (robot_p - goal_p).norm() < 0.1;
        if (!flying && near_robot) {
            return;
        }
        const bool hold = flying && near_robot;
        // While braking the drone is already stopping: a hold request changes nothing.
        if (hold && state == BRAKE) {
            return;
        }

        // Held until the goal is stored: gi_, finish_plan and goal_seq_ change together (see
        // goal_mutex_). closeToGoal() also takes goal_mutex_, so it is not called below.
        std::unique_lock<std::mutex> lck(goal_mutex_);

        // Moving-goal gate. While the current goal is being flown to, or has been reached, a goal
        // within regoal_dist of it is ignored entirely (goal, yaw and new_goal untouched), so a
        // 10 Hz goal stream that barely moves forces no replans (an accepted goal sets new_goal,
        // which makes ReplanOnce replan instead of keeping the current trajectory). Goals are
        // compared as received, before snapping: near obstacles the snap can move two nearby
        // goals apart by more than regoal_dist. The comparison is always against the last
        // accepted goal, so slow drift is accepted once it adds up past regoal_dist. Not gated:
        // the first goal, and WAIT_GOAL away from the goal (after an invalid goal or an
        // emergency stop), so a repeated goal there retries planning, and hold requests.
        const bool goal_active = flying || (state == WAIT_GOAL &&
                                            (robot_p - gi_.goal_p).norm() < cfg_.regoal_dist);
        if (!hold && started_ && goal_active && (click_point - gi_.click_p).norm() <= cfg_.regoal_dist) {
            return;
        }
        // A hold request within 0.2 m (twice the hold radius) of the current goal changes nothing:
        // the drone already stops there. Ignoring it keeps a goal stream that sits on the drone
        // while it arrives at its goal (still FOLLOW_TRAJ) from forcing a replan per message.
        if (hold && started_ && (goal_p - gi_.goal_p).norm() <= 0.2) {
            return;
        }

        gi_.click_p = click_point;
        gi_.goal_p = goal_p;
        cout << GREEN << " -- [Fsm] Get goal at " << RESET << gi_.goal_p.transpose() << endl;

        if (cfg_.click_yaw_en) {
            if (isnan(q.w()) || isnan(q.x()) || isnan(q.y()) || isnan(q.z())) {
                gi_.goal_yaw = NAN;
                ros_ptr_->info(" -- [Fsm] Receive click goal at: [{}, {}, {}]; goal yaw disabled",
                               gi_.goal_p.x(), gi_.goal_p.y(), gi_.goal_p.z());
            } else {
                gi_.goal_yaw = geometry_utils::get_yaw_from_quaternion(q);
                cout << GREEN << " -- [Fsm] Receive click goal at: [" << gi_.goal_p.transpose() << "]; goal yaw: "
                     << gi_.goal_yaw * 57.3 << " deg" << RESET << endl;
            }

        } else {
            gi_.goal_yaw = NAN;
            cout << GREEN << " -- [Fsm] Receive click goal at: [" << gi_.goal_p.transpose() << "]; goal yaw disabled"
                 << RESET << endl;
        }

        started_ = true;
        gi_.new_goal = true;
        // finish_plan (set when PlanFromRest already reached the goal) stops callReplanOnce. The
        // goal has moved, so resume replanning toward it.
        finish_plan = false;
        goal_seq_++;
        const uint64_t seq = goal_seq_;
        lck.unlock();

        // Hold brake. An accepted hold while a trajectory is flown (FOLLOW_TRAJ) means "stop
        // now": upstream would replan toward the hold point, and while replans fail it keeps
        // flying the committed trajectory at speed. Instead the rest of the trajectory is replaced
        // by a brake along it, and the drone hovers where it stops (not at the hold point, which
        // it has passed by then; see finishBrake). In GENERATE_TRAJ nothing is flown, so the hold
        // is planned from rest as upstream.
        if (hold) {
            startBrake("hold", seq, true);
        }
    }

    bool Fsm::startBrake(const char *trigger, const uint64_t &seq, const bool &hold) {
        if (!brakeSupported()) {
            return false;
        }
        std::lock_guard<std::mutex> lck(brake_mutex_);
        if (machine_state_ != FOLLOW_TRAJ) {
            return false;
        }
        double speed, stop_dist, stop_time;
        Vec3f stop_p;
        if (!planner_ptr_->prepareBrake(ros_ptr_->getSimTime(), speed, stop_p, stop_dist, stop_time)) {
            return false;
        }
        brake_seq_ = seq;
        brake_for_hold_ = hold;
        if (!changeStateFrom("startBrake", FOLLOW_TRAJ, BRAKE)) {
            return false;
        }
        ros_ptr_->info(" -- [Fsm] BRAKE ({}): {:.2f} m/s, stops in {:.2f} m and {:.2f} s at [{:.2f}, {:.2f}, {:.2f}].",
                       trigger, speed, stop_dist, stop_time, stop_p.x(), stop_p.y(), stop_p.z());
        return true;
    }

    void Fsm::finishBrake() {
        const Vec3f stop_p = planner_ptr_->getHoldPoint();
        bool hover_at_stop = false;
        {
            std::lock_guard<std::mutex> lck(goal_mutex_);
            if (brake_for_hold_ && brake_seq_ == goal_seq_) {
                gi_.goal_p = stop_p;
                gi_.new_goal = false;
                finish_plan = true;
                hover_at_stop = true;
            }
        }
        hold_cmd_valid_ = true;
        changeStateFrom("finishBrake", BRAKE, WAIT_GOAL);
        ros_ptr_->info(" -- [Fsm] brake done at [{:.2f}, {:.2f}, {:.2f}], {}.", stop_p.x(), stop_p.y(), stop_p.z(),
                       hover_at_stop ? "hovering there" : "planning from rest to the latest goal");
    }

    bool Fsm::changeStateFrom(const string &call_func, MACHINE_STATE expected, const MACHINE_STATE &new_state) {
        const MACHINE_STATE from = expected;
        if (!machine_state_.compare_exchange_strong(expected, new_state)) {
            return false;
        }
        // As ChangeState. Cleared just after the swap rather than before it: the hover is only
        // sent in WAIT_GOAL, which none of these states is.
        if (new_state == FOLLOW_TRAJ || new_state == EMER_STOP || new_state == INIT) {
            hold_cmd_valid_ = false;
        }
        fmt::print(fg(fmt::color::green), " -- [Fsm]: [{}] change state from [{}] to [{}].\n", call_func,
                   MACHINE_STATE_STR[int(from)], MACHINE_STATE_STR[int(new_state)]);
        return true;
    }

    void Fsm::ChangeState(const string &call_func, const MACHINE_STATE &new_state) {
        fmt::print(fg(fmt::color::green), " -- [Fsm]: [{}] change state from [{}] to [{}].\n", call_func,
                   MACHINE_STATE_STR[int(machine_state_.load())], MACHINE_STATE_STR[int(new_state)]);
        if (new_state == FOLLOW_TRAJ || new_state == EMER_STOP || new_state == INIT) {
            hold_cmd_valid_ = false;
        }
        machine_state_ = new_state;
    }
}
