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


#ifndef SUPER_FSM_CONFIG_HPP
#define SUPER_FSM_CONFIG_HPP


#include <super_core/config.hpp>
#include <vector>
#include <cstring>
#include <limits>
#include <utils/header/yaml_loader.hpp>

namespace fsm {
    using namespace traj_opt;
    using namespace super_planner;
    static constexpr int MPC_PVAJ_MODE = 1;
    static constexpr int MPC_POLYTRAJ_MODE = 2;

    class Config {
    public:
        bool timer_en{true};

        // Fsm Params
        bool click_goal_en{},visualization_en{};
        double replan_rate{}, resolution{};
        double click_height{};
        // A new goal within this distance [m] of the current goal is ignored while that goal is
        // being flown to or has been reached (see Fsm::setGoalPosiAndYaw).
        double regoal_dist{};

        bool click_yaw_en{};
        string cmd_topic, mpc_cmd_topic, click_goal_topic;
        // geometry_msgs/PointStamped (world frame) the yaw looks at; subscribed only with super_planner/yaw_mode 3.
        string yaw_target_topic;
        double yaw_dot_max{};
        // yaw_mode 3 hover command: maximum odometry age [s] (time since the last odometry message
        // arrived). Older odometry suppresses the hover (see FsmRos1::pubCmdTimerCallback). No
        // usable default: NaN when missing, and FsmRos1::init exits if yaw_mode 3 is set without a
        // finite value > 0.
        double hover_max_odom_age{std::numeric_limits<double>::quiet_NaN()};
        // yaw_mode 3 hover command: maximum distance [m] between the robot and the latched hover
        // point; farther suppresses the hover. Same rules as hover_max_odom_age (NaN when missing,
        // FsmRos1::init exits with yaw_mode 3 unless finite and > 0).
        double hover_max_offset{std::numeric_limits<double>::quiet_NaN()};
        // Brake when replans fail for longer than this [s] while the goal differs from the one the
        // committed trajectory was planned for (see Fsm::callReplanOnce). No usable default: NaN
        // when missing, and FsmRos1::init exits unless finite and > 0.
        double brake_after_replan_fail{std::numeric_limits<double>::quiet_NaN()};

        Config() = default;

        Config(const std::string & cfg_path) {
            yaml_loader::YamlLoader loader(cfg_path);
            vector<double> tem_gain;
            loader.LoadParam("fsm/timer_en", timer_en, false);
            loader.LoadParam("fsm/click_goal_en", click_goal_en, false);
            loader.LoadParam("fsm/click_yaw_en", click_yaw_en, false);
            loader.LoadParam("fsm/replan_rate", replan_rate, 10.0);
            loader.LoadParam("fsm/click_height", click_height, 1.5);
            loader.LoadParam("fsm/regoal_dist", regoal_dist, 0.5);
            loader.LoadParam("fsm/cmd_topic", cmd_topic, string("/planning/pos_cmd"));
            loader.LoadParam("fsm/mpc_cmd_topic", mpc_cmd_topic, string("/planning_cmd/mpc"));
            loader.LoadParam("fsm/click_goal_topic", click_goal_topic, string("/planning/click_goal_topic"));
            loader.LoadParam("fsm/yaw_target_topic", yaw_target_topic, string("/planning/yaw_target"));
            loader.LoadParam("fsm/hover_max_odom_age", hover_max_odom_age,
                             std::numeric_limits<double>::quiet_NaN());
            loader.LoadParam("fsm/hover_max_offset", hover_max_offset,
                             std::numeric_limits<double>::quiet_NaN());
            loader.LoadParam("fsm/brake_after_replan_fail", brake_after_replan_fail,
                             std::numeric_limits<double>::quiet_NaN());


            loader.LoadParam("super_planner/yaw_dot_max", yaw_dot_max, 1.0, true);
            loader.LoadParam("super_planner/visualization_en", visualization_en, false, true);
            loader.LoadParam("rog_map/resolution", resolution, 0.1, true);

        }
    };
}

#endif //SUPER_FSM_CONFIG_H
