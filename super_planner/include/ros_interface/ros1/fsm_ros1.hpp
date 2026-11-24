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


#ifdef USE_ROS1

#ifndef SRC_FSM_ROS1_HPP
#define SRC_FSM_ROS1_HPP

#include "fsm/fsm.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <utility>

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"
#include "quadrotor_msgs/PositionCommand.h"
#include "quadrotor_msgs/PolynomialTrajectory.h"


namespace fsm {
    class FsmRos1 : public Fsm {
        ros::NodeHandle nh_;
        ros::Subscriber goal_sub_, yaw_target_sub_;
        ros::Publisher cmd_pub, mpc_cmd_pub_, path_pub_;
        ros::Timer execution_timer_, replan_timer_, cmd_timer_;
        quadrotor_msgs::PositionCommand pid_cmd_;
        rog_map::ROGMapROS::Ptr map_ptr_;
        quadrotor_msgs::PositionCommand latest_cmd;
        nav_msgs::Path path;

        vector<quadrotor_msgs::PositionCommand> cmd_logs_;

        void resetVisualizedPath() override {
            path.poses.clear();
        }

        void publishCurPoseToPath() override {
            path.header.frame_id = "world";
            path.header.stamp = ros::Time::now();
            geometry_msgs::PoseStamped pose;
            pose.header = path.header;
            pose.pose.position.x = robot_state_.p(0);
            pose.pose.position.y = robot_state_.p(1);
            pose.pose.position.z = robot_state_.p(2);
            pose.pose.orientation.x = robot_state_.q.x();
            pose.pose.orientation.y = robot_state_.q.y();
            pose.pose.orientation.z = robot_state_.q.z();
            pose.pose.orientation.w = robot_state_.q.w();
            path.poses.push_back(pose);
            path_pub_.publish(path);
        }

        void publishPolyTraj() override {
            quadrotor_msgs::PolynomialTrajectory cmd_traj;
            getCommittedTrajectory(cmd_traj);
            mpc_cmd_pub_.publish(cmd_traj);
        }

        void getOneHeartBeatMsg(quadrotor_msgs::PolynomialTrajectory &heartbeat, bool &traj_finish) {
            heartbeat.type = quadrotor_msgs::PolynomialTrajectory::HEART_BEAT;
            heartbeat.header.stamp = ros::Time::now();
            heartbeat.header.frame_id = "world";
            double swt;
            planner_ptr_->getOneHeartbeatTime(swt, traj_finish);
            heartbeat.start_WT_pos = ros::Time(swt);
        }

        void getCommittedTrajectory(quadrotor_msgs::PolynomialTrajectory &cmd_traj) {
            cmd_traj.header.stamp = ros::Time::now();
            cmd_traj.header.frame_id = "world";
            cmd_traj.type = quadrotor_msgs::PolynomialTrajectory::POSITION_TRAJ |
                            quadrotor_msgs::PolynomialTrajectory::HEART_BEAT;
            planner_ptr_->lockCommittedTraj();
            const Trajectory pos_traj = planner_ptr_->getCommittedPositionTrajectory();
            const Trajectory yaw_traj = planner_ptr_->getCommittedYawTrajectory();
            planner_ptr_->unlockCommittedTraj();

            cmd_traj.start_WT_pos = ros::Time(pos_traj.start_WT);

            cmd_traj.piece_num_pos = pos_traj.getPieceNum();
            cmd_traj.order_pos = 7;
            cmd_traj.time_pos.resize(pos_traj.getPieceNum());
            cmd_traj.coef_pos_x.resize(cmd_traj.piece_num_pos * (cmd_traj.order_pos + 1));
            cmd_traj.coef_pos_y.resize(cmd_traj.piece_num_pos * (cmd_traj.order_pos + 1));
            cmd_traj.coef_pos_z.resize(cmd_traj.piece_num_pos * (cmd_traj.order_pos + 1));
            cmd_traj.start_WT_pos = ros::Time(pos_traj.start_WT);

            if (!yaw_traj.empty()) {
                cmd_traj.type = cmd_traj.type |
                                quadrotor_msgs::PolynomialTrajectory::YAW_TRAJ;
                cmd_traj.piece_num_yaw = yaw_traj.getPieceNum();
                cmd_traj.order_yaw = 7;
                double col_size = cmd_traj.order_yaw + 1;
                cmd_traj.coef_yaw.resize(cmd_traj.piece_num_yaw * col_size);
                cmd_traj.time_yaw.resize(cmd_traj.piece_num_yaw);
                for (int i = 0; i < cmd_traj.piece_num_yaw; i++) {
                    Eigen::VectorXd yaw_coef = yaw_traj[i].getCoeffMat().row(0);
                    Eigen::Map<Eigen::VectorXd>(&cmd_traj.coef_yaw[col_size * i], col_size) = yaw_coef;
                    cmd_traj.time_yaw[i] = yaw_traj[i].getDuration();
                }
                cmd_traj.start_WT_yaw = ros::Time(yaw_traj.start_WT);
            }

            for (int i = 0; i < cmd_traj.piece_num_pos; i++) {
                Eigen::Matrix<double, 3, 8> coef = pos_traj[i].getCoeffMat();
                Eigen::Map<Eigen::VectorXd>(&cmd_traj.coef_pos_x[8 * i], 8) = coef.row(0);
                Eigen::Map<Eigen::VectorXd>(&cmd_traj.coef_pos_y[8 * i], 8) = coef.row(1);
                Eigen::Map<Eigen::VectorXd>(&cmd_traj.coef_pos_z[8 * i], 8) = coef.row(2);
                cmd_traj.time_pos[i] = pos_traj[i].getDuration();
            }
        }

        void getOnePositionCommand(quadrotor_msgs::PositionCommand &pos_cmd, bool &traj_finish) {
            StatePVAJ pvaj;
            double yaw, yaw_dot;
            bool on_backup_traj;
            planner_ptr_->getOneCommandFromTraj(pvaj, yaw, yaw_dot, on_backup_traj, traj_finish);
            fillPositionCommand(pvaj, yaw, yaw_dot, on_backup_traj, pos_cmd);
        }

        bool brakeSupported() const override {
            return true;
        }

        /// yaw_mode 3 hover command, see pubCmdTimerCallback.
        void getHoldPositionCommand(quadrotor_msgs::PositionCommand &pos_cmd) {
            StatePVAJ pvaj;
            double yaw, yaw_dot;
            planner_ptr_->getHoldCommand(pvaj, yaw, yaw_dot);
            // Not added to cmd_logs_: hovering can last indefinitely and the log is kept in memory.
            fillPositionCommand(pvaj, yaw, yaw_dot, false, pos_cmd, false);
        }

        void fillPositionCommand(const StatePVAJ &pvaj, const double &yaw, const double &yaw_dot,
                                 const bool &on_backup_traj, quadrotor_msgs::PositionCommand &pos_cmd,
                                 const bool &log = true) {
            pos_cmd.trajectory_flag = 0;
            pos_cmd.header.stamp = ros::Time::now();
            pos_cmd.header.frame_id = "world";
            pos_cmd.position.x = pvaj(0, 0);
            pos_cmd.position.y = pvaj(1, 0);
            pos_cmd.position.z = pvaj(2, 0);
            pos_cmd.velocity.x = pvaj(0, 1);
            pos_cmd.velocity.y = pvaj(1, 1);
            pos_cmd.velocity.z = pvaj(2, 1);
            pos_cmd.acceleration.x = pvaj(0, 2);
            pos_cmd.acceleration.y = pvaj(1, 2);
            pos_cmd.acceleration.z = pvaj(2, 2);
            pos_cmd.jerk.x = pvaj(0, 3);
            pos_cmd.jerk.y = pvaj(1, 3);
            pos_cmd.jerk.z = pvaj(2, 3);
            pos_cmd.yaw = yaw;
            pos_cmd.yaw_dot = yaw_dot;
            pos_cmd.trajectory_flag = on_backup_traj ? 2 : 1;
            Vec3f rpy, omg;
            double aT;
            geometry_utils::convertFlatOutputToAttAndOmg(pvaj.col(0), pvaj.col(1), pvaj.col(2), pvaj.col(3), yaw,
                                                         yaw_dot, rpy, omg, aT);
            pos_cmd.attitude.x = rpy(0);
            pos_cmd.attitude.y = rpy(1);
            pos_cmd.attitude.z = rpy(2);
            pos_cmd.angular_velocity.x = omg(0);
            pos_cmd.angular_velocity.y = omg(1);
            pos_cmd.angular_velocity.z = omg(2);
            pos_cmd.thrust.z = aT;
            latest_cmd = pos_cmd;
            if (log) {
                cmd_logs_.push_back(latest_cmd);
            }
        }

    public:
        FsmRos1() = default;

        ~FsmRos1(){
            ros::shutdown();
            saveReplanLogToFile("super_latest_log");
            exit(0);
        };

        typedef std::shared_ptr<FsmRos1> Ptr;

        void saveReplanLogToFile(const string &name = "") {
            // run statistic
            double total_length{0.0};
            int total_replan_num{0};
            double average_compt_t{0.0};
            Vec3f cur_p{0, 0, 0};
            for (auto rp: replan_logs_) {
                if (rp.getRetCode() > 0) {
                    if (cur_p.norm() < 1e-6) {
                        cur_p = rp.getRobotP();
                    } else {
                        total_length += (rp.getRobotP() - cur_p).norm();
                        cur_p = rp.getRobotP();
                    }
                    total_replan_num++;
                    average_compt_t += rp.getTotalCompT();
                }
            }


            fmt::print("Total replan num: {}, total length: {}, average computation time: {} ms\n",
                       total_replan_num, total_length, average_compt_t / (total_replan_num==0?1:total_replan_num) * 1000);


            const std::string save_path = name.empty()
                                          ? LOG_FILE_DIR(
                                                  "replan_logs/" + BinaryFileHandler<int>::getCurrentTimeStr() + ".bin")
                                          : LOG_FILE_DIR("replan_logs/" + name + ".bin");
            const std::string csv_path = name.empty()
                                         ? LOG_FILE_DIR(
                                                 "cmd_logs/" + BinaryFileHandler<int>::getCurrentTimeStr() + ".csv")
                                         : LOG_FILE_DIR("cmd_logs/" + name + ".csv");
            BinaryFileHandler<vector<LogOneReplan>>::save(save_path, replan_logs_);

            std::ofstream csv_writer;
            csv_writer.open(csv_path, std::ios::out | std::ios::trunc);
            csv_writer
                    << "time,posi_x,posi_y,posi_z,vel_x,vel_y,vel_z,acc_x,acc_y,acc_z,jerk_x,jerk_y,jerk_z,yaw,yaw_rate,backup"
                    << std::endl;
            csv_writer<<std::fixed<<std::setprecision(15);
            for (const auto &cmd: cmd_logs_) {
                csv_writer << cmd.header.stamp.toSec() - system_start_time_ << "," << cmd.position.x << "," << cmd.position.y << ","
                           << cmd.position.z << ","
                           << cmd.velocity.x << "," << cmd.velocity.y << "," << cmd.velocity.z << ","
                           << cmd.acceleration.x << "," << cmd.acceleration.y << "," << cmd.acceleration.z << ","
                           << cmd.jerk.x << "," << cmd.jerk.y << "," << cmd.jerk.z << ","
                           << cmd.yaw << "," << cmd.yaw_dot << "," << static_cast<int>(cmd.trajectory_flag)
                           << std::endl;
            }
            csv_writer.close();
        }

        bool getPoseFromTraj(super_utils::Pose &pose) {
            if (machine_state_ != FOLLOW_TRAJ) {
                cout << YELLOW << "[Fsm] Not in FOLLOW_TRAJ state, can't get pose from traj." << RESET << endl;
                return false;
            }
            getOnePositionCommand(pid_cmd_, traj_finish_);
            if (traj_finish_) {
                cout << GREEN << " -- [Fsm] Traj finish." << RESET << endl;
                if (machine_state_ == FOLLOW_TRAJ) {
                    planner_ptr_->latchHoldPoint();
                    hold_cmd_valid_ = true;
                }
                if (closeToGoal(0.1)) {
                    ChangeState("getPoseFromTraj", WAIT_GOAL);
                } else {
                    ChangeState("getPoseFromTraj", GENERATE_TRAJ);
                }
            }
            pose.first = Vec3f{pid_cmd_.position.x, pid_cmd_.position.y, pid_cmd_.position.z};
            pose.second = eulerToQuaternion(pid_cmd_.attitude.x, pid_cmd_.attitude.y, pid_cmd_.attitude.z);


            /// for checking the trajectory continuty
            static double max_delta_v{0.0};
            static double last_v = pid_cmd_.vel_norm;
            double delta_v = std::abs(pid_cmd_.vel_norm - last_v);
            last_v = pid_cmd_.vel_norm;
            if (delta_v > max_delta_v) {
                max_delta_v = delta_v;
            }
            fmt::print(" -- [Fsm] Cur vel: {}, delta_v: {}, max_delta_v: {}\n", pid_cmd_.vel_norm, delta_v,
                       max_delta_v);
            cmd_logs_.push_back(latest_cmd);
            return true;
        }

        void goalCallback(const geometry_msgs::PoseStampedConstPtr &msg) {
            super_utils::Vec3f goal_p = Vec3f{msg->pose.position.x, msg->pose.position.y, msg->pose.position.z};
            super_utils::Quatf goal_q = super_utils::Quatf{msg->pose.orientation.w, msg->pose.orientation.x,
                                                           msg->pose.orientation.y, msg->pose.orientation.z};
            setGoalPosiAndYaw(goal_p, goal_q);
        }

        void yawTargetCallback(const geometry_msgs::PointStampedConstPtr &msg) {
            // A zero stamp means "now". Staleness is judged in SuperPlanner::applyLookAtYaw.
            const double stamp = msg->header.stamp.isZero() ? ros::Time::now().toSec() : msg->header.stamp.toSec();
            planner_ptr_->setYawTarget(Vec3f{msg->point.x, msg->point.y, msg->point.z}, stamp);
        }

        void init(const ros::NodeHandle &nh, const std::string &cfg_path) {
            // 初始化参数读取
            nh_ = nh;
            cfg_ = Config(cfg_path);
            map_ptr_ = std::make_shared<rog_map::ROGMapROS>(nh, cfg_path);
            // 初始化Planner
            ros_ptr_ = std::make_shared<ros_interface::Ros1Interface>(nh_);
            planner_ptr_ = std::make_shared<SuperPlanner>(cfg_path, ros_ptr_, map_ptr_);
            cmd_pub = nh_.advertise<quadrotor_msgs::PositionCommand>(cfg_.cmd_topic, 10);
            mpc_cmd_pub_ = nh_.advertise<quadrotor_msgs::PolynomialTrajectory>(cfg_.mpc_cmd_topic, 10);
            path_pub_ = nh_.advertise<nav_msgs::Path>("fsm/path", 100);

            int cmd_cnt = 0;

            if (cfg_.click_goal_en) {
                goal_sub_ = nh_.subscribe(cfg_.click_goal_topic, 1, &FsmRos1::goalCallback, this);
                cout << YELLOW << " -- [Fsm] CLICKGOAL ENABLE." << RESET << endl;
                cmd_cnt++;
            }

            if (planner_ptr_->yawToPointEnabled()) {
                // The hover command needs its guard limits; never guess them. A missing or
                // mistyped parameter is NaN. _Exit as in SuperPlanner's config check.
                const std::pair<const char *, double> hover_limits[] = {
                        {"fsm/hover_max_odom_age", cfg_.hover_max_odom_age},
                        {"fsm/hover_max_offset", cfg_.hover_max_offset}};
                for (const auto &limit : hover_limits) {
                    if (!std::isfinite(limit.second) || !(limit.second > 0)) {
                        ROS_FATAL(" -- [Fsm] invalid config: %s = %f must be set, finite and > 0 "
                                  "with super_planner/yaw_mode 3. Exiting.", limit.first, limit.second);
                        std::cout.flush();
                        std::fflush(stdout);
                        std::_Exit(EXIT_FAILURE);
                    }
                }
                yaw_target_sub_ = nh_.subscribe(cfg_.yaw_target_topic, 1, &FsmRos1::yawTargetCallback, this);
                cout << YELLOW << " -- [Fsm] YAW TO POINT ENABLE, target topic: " << cfg_.yaw_target_topic << RESET << endl;
            }

            if (cmd_cnt != 1) {
                cout << YELLOW << " -- [Fsm] CMD INPUT ERROR." << RESET << endl;
                exit(0);
            }

            if (cfg_.timer_en) {
                execution_timer_ = nh_.createTimer(ros::Duration(0.01), &FsmRos1::mainFsmTimerCallback, this); // 100Hz
                cmd_timer_ = nh_.createTimer(ros::Duration(0.01), &FsmRos1::pubCmdTimerCallback, this); // 100Hz
                replan_timer_ = nh_.createTimer(ros::Duration(1.0 / cfg_.replan_rate), &FsmRos1::replanTimerCallback,
                                                this); // 10Hz
            }

            write_time_.open(DEBUG_FILE_DIR("time_consuming.csv"), std::ios::out | std::ios::trunc);
            log_module_time.resize(9);
            for (int i = 0; i < 9; i++) {
                write_time_ << log_time_str[i];
                if (i != 8) {
                    write_time_ << ",";
                }
            }
            write_time_ << endl;
            machine_state_ = INIT;
            system_start_time_ = ros_ptr_->getSimTime();

            pid_cmd_.kx[0] = 5.7;
            pid_cmd_.kx[1] = 5.7;
            pid_cmd_.kx[2] = 4.2;

            pid_cmd_.kv[0] = 3.4;
            pid_cmd_.kv[1] = 3.4;
            pid_cmd_.kv[2] = 4.0;
        }

        /// 100 Hz command output. What the drone is told in each state:
        /// - INIT: nothing is published; nothing has been planned yet.
        /// - FOLLOW_TRAJ: the committed trajectory, sampled now (pos_cmd plus a heartbeat on
        ///   mpc_cmd_topic). With yaw_mode 3, yaw and yaw_dot come from look-at.
        /// - EMER_STOP: as FOLLOW_TRAJ (upstream: the committed trajectory, which contains the
        ///   backup/stop segment), for the one FSM tick before it changes to WAIT_GOAL.
        /// - BRAKE: the brake profile (SuperPlanner::getBrakeCommand), on pos_cmd only: nothing on
        ///   mpc_cmd_topic, so a controller tracking the polynomial trajectory there is not
        ///   braked. When the profile ends, the last command is the stop point at zero velocity,
        ///   the stop point is latched as the hover point and finishBrake changes to WAIT_GOAL:
        ///   the hover above (yaw_mode 3), or nothing published (yaw_mode 1/2, or a newer goal
        ///   planned from rest in GENERATE_TRAJ), so the controller keeps that last command.
        /// - WAIT_GOAL after a trajectory was flown to its end, yaw_mode 3 only: a hover command at
        ///   the trajectory's final point, latched when it was reached (never read again from the
        ///   live trajectory), with zero velocity, acceleration and jerk, and the look-at yaw, or
        ///   the last commanded yaw held once the target is stale. Only pos_cmd; nothing on
        ///   mpc_cmd_topic. This includes WAIT_GOAL reached through GENERATE_TRAJ (trajectory end
        ///   0.1 m or more from the goal, or a later goal that was invalid): nothing new was
        ///   committed and the drone is still at the latched point. If odometry has not been
        ///   received or is older than fsm/hover_max_odom_age, or the robot is not within
        ///   fsm/hover_max_offset of the latched point (including a NaN offset), nothing is
        ///   published (throttled warning): the controller keeps its last setpoint (px4_bridge/PX4
        ///   hold).
        /// - WAIT_GOAL when no trajectory has been flown to its end since the last EMER_STOP or
        ///   new trajectory (e.g. right after EMER_STOP, or after a NEW_TRAJ replan mid-flight that
        ///   ended at the goal through GENERATE_TRAJ), WAIT_GOAL with yaw_mode 1/2, and
        ///   GENERATE_TRAJ (including while a failed PlanFromRest retries there): nothing is
        ///   published (upstream); the controller keeps its last setpoint. After EMER_STOP the
        ///   trajectory's final point is not where the drone stopped.
        /// machine_state_ is checked and changed here without a lock around the sequence, as
        /// upstream. The replan timer may switch FOLLOW_TRAJ -> EMER_STOP or FOLLOW_TRAJ ->
        /// GENERATE_TRAJ (NEW_TRAJ) between the check and hold_cmd_valid_ = true below. Both are
        /// benign because the trajectory has finished, so the drone is at the latched final point:
        /// - EMER_STOP: ReplanOnce committed nothing; this timer then sets WAIT_GOAL or
        ///   GENERATE_TRAJ (upstream does the same), and the flag may stay set, so the hover is
        ///   published at that point in WAIT_GOAL.
        /// - GENERATE_TRAJ: this timer then sets WAIT_GOAL (hover at that point) or GENERATE_TRAJ
        ///   (nothing published) until a new trajectory enters FOLLOW_TRAJ and clears the flag.
        /// Should a new trajectory be committed inside that window, the latched point may not be
        /// where the drone is; the odometry and offset checks below then suppress the hover.
        void pubCmdTimerCallback(const ros::TimerEvent &event) {
            if (stop) {
                return;
            }
            if (machine_state_ == WAIT_GOAL && hold_cmd_valid_ && planner_ptr_->yawToPointEnabled()) {
                rog_map::RobotState rs;
                {
                    std::lock_guard<std::mutex> lck(robot_state_mutex_);
                    rs = robot_state_;  // p and rcv_time from the same odometry sample
                }
                if (!rs.rcv) {
                    ROS_WARN_THROTTLE(1.0, " -- [Fsm] no odometry received, hover command not published.");
                    return;
                }
                const double odom_age = ros_ptr_->getSimTime() - rs.rcv_time;
                if (!(odom_age <= cfg_.hover_max_odom_age)) {
                    ROS_WARN_THROTTLE(1.0, " -- [Fsm] odometry %.3f s old (> %.2f s), "
                                           "hover command not published.", odom_age, cfg_.hover_max_odom_age);
                    return;
                }
                const double offset = (planner_ptr_->getHoldPoint() - rs.p).norm();
                if (!(offset <= cfg_.hover_max_offset)) {
                    ROS_WARN_THROTTLE(1.0, " -- [Fsm] hover point %.2f m from the robot (> %.2f m), "
                                           "hover command not published.", offset, cfg_.hover_max_offset);
                    return;
                }
                getHoldPositionCommand(pid_cmd_);
                cmd_pub.publish(pid_cmd_);
                return;
            }
            if (machine_state_ == BRAKE) {
                StatePVAJ pvaj;
                double yaw, yaw_dot;
                bool finished;
                planner_ptr_->getBrakeCommand(pvaj, yaw, yaw_dot, finished);
                fillPositionCommand(pvaj, yaw, yaw_dot, false, pid_cmd_);
                cmd_pub.publish(pid_cmd_);
                if (finished) {
                    planner_ptr_->latchBrakeStopPoint();
                    finishBrake();
                }
                return;
            }
            const MACHINE_STATE observed = machine_state_;
            if (observed != FOLLOW_TRAJ && observed != EMER_STOP) {
                return;
            }


            quadrotor_msgs::PolynomialTrajectory heartbeat;
            getOneHeartBeatMsg(heartbeat, traj_finish_);
            getOnePositionCommand(pid_cmd_, traj_finish_);
            mpc_cmd_pub_.publish(heartbeat);
            cmd_pub.publish(pid_cmd_);
            if (traj_finish_) {
                cout << GREEN << " -- [Fsm] Traj finish." << RESET << endl;
                if (machine_state_ == FOLLOW_TRAJ) {
                    planner_ptr_->latchHoldPoint();
                    hold_cmd_valid_ = true;
                }
                // From the state seen above only: a brake started meanwhile stays in place (it
                // stops at the trajectory end at the latest, then finishBrake).
                changeStateFrom("PubCmdCallback", observed, closeToGoal(0.1) ? WAIT_GOAL : GENERATE_TRAJ);
            }
        }

        void replanTimerCallback(const ros::TimerEvent &event) {
            callReplanOnce();
        }

        void mainFsmTimerCallback(const ros::TimerEvent &event) {
            callMainFsmOnce();
        }

    };
}

#endif //SRC_FSM_ROS1_HPP

#endif