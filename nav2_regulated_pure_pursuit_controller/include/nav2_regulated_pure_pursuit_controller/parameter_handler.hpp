// Copyright (c) 2022 Samsung Research America
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAV2_REGULATED_PURE_PURSUIT_CONTROLLER__PARAMETER_HANDLER_HPP_
#define NAV2_REGULATED_PURE_PURSUIT_CONTROLLER__PARAMETER_HANDLER_HPP_

#include <algorithm>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/odometry_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace nav2_regulated_pure_pursuit_controller
{

struct Parameters
{
    // 期望的线性速度，用于控制机器人的前进速度
    double desired_linear_vel;

    // 基准期望线性速度，可用于调整期望线性速度的基准值
    double base_desired_linear_vel;

    // 线性前瞻距离，用于计算机器人在路径上的前瞻点
    double lookahead_dist;

    // 旋转到目标方向的角速度，用于控制机器人的角速度
    double rotate_to_heading_angular_vel;

    // 最大前瞻距离，限制了前瞻点与机器人的最大距离
    double max_lookahead_dist;

    // 最小前瞻距离，确保前瞻点不会太靠近机器人
    double min_lookahead_dist;

    // 前瞻时间，用于计算前瞻点，与前瞻距离一起使用
    double lookahead_time;

    // 是否使用基于速度缩放的前瞻距离
    bool use_velocity_scaled_lookahead_dist;

    // 最小接近线性速度，用于在接近目标时缩小机器人的速度
    double min_approach_linear_velocity;

    // 接近速度缩放距离，用于在接近目标时调整机器人速度的缩放
    double approach_velocity_scaling_dist;

    // 允许的最大时间到碰撞点，用于调整避免碰撞的策略
    double max_allowed_time_to_collision_up_to_carrot;

    // 是否使用受控的线性速度缩放
    bool use_regulated_linear_velocity_scaling;

    // 是否使用成本受控的线性速度缩放
    bool use_cost_regulated_linear_velocity_scaling;

    // 成本缩放距离，用于基于成本来调整速度缩放
    double cost_scaling_dist;

    // 成本缩放增益，用于调整成本缩放的强度
    double cost_scaling_gain;

    // 充气成本缩放因子，用于调整基于障碍物充气成本的影响
    double inflation_cost_scaling_factor;

    // 受控线性缩放的最小半径，用于限制受控缩放的生效范围
    double regulated_linear_scaling_min_radius;

    // 受控线性缩放的最小速度，用于限制受控缩放的生效范围
    double regulated_linear_scaling_min_speed;

    // 是否使用固定曲率的前瞻点距离
    bool use_fixed_curvature_lookahead;

    // 曲率前瞻距离，用于计算基于曲率的前瞻点距离
    double curvature_lookahead_dist;

    // 是否使用旋转到目标方向的控制
    bool use_rotate_to_heading;

    // 最大角加速度，用于限制机器人的旋转速度变化率
    double max_angular_accel;

    // 旋转到目标方向的最小角度，用于判断是否需要进行旋转
    double rotate_to_heading_min_angle;

    // 是否允许后退运动
    bool allow_reversing;

    // 最大机器人姿态搜索距离，用于确定搜索全局路径上的机器人最近的点
    double max_robot_pose_search_dist;

    // 是否使用插值法在路径上插值
    bool use_interpolation;

    // 是否使用碰撞检测来避免碰撞
    bool use_collision_detection;

    // 坐标变换的容忍度，用于在坐标转换时控制误差
    double transform_tolerance;
};

/**
 * @class nav2_regulated_pure_pursuit_controller::ParameterHandler
 * @brief Handles parameters and dynamic parameters for RPP
 */
class ParameterHandler
{
public:
    /**
     * @brief Constructor for nav2_regulated_pure_pursuit_controller::ParameterHandler
     */
    ParameterHandler(
        rclcpp_lifecycle::LifecycleNode::SharedPtr node,
        std::string &plugin_name,
        rclcpp::Logger &logger, const double costmap_size_x);

    /**
     * @brief Destrructor for nav2_regulated_pure_pursuit_controller::ParameterHandler
     */
    ~ParameterHandler() = default;

    std::mutex &getMutex() { return mutex_; }

    Parameters *getParams() { return &params_; }

protected:
    /**
     * @brief Callback executed when a parameter change is detected 当检测到参数更改时执行的回调函数 // 定义了一个函数，用于处理动态参数的回调，并返回参数设置的结果
     * @param event ParameterEvent message
     */
    rcl_interfaces::msg::SetParametersResult
    dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

    // Dynamic parameters handler 动态参数处理
    std::mutex mutex_;
    // 这是一个动态参数回调句柄的智能指针。它用于管理注册的动态参数回调函数。当参数发生变化时，回调函数将被调用。
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_; // 动态参数回调句柄
    Parameters params_;
    std::string plugin_name_;
    rclcpp::Logger logger_{rclcpp::get_logger("RegulatedPurePursuitController")};
};

} // namespace nav2_regulated_pure_pursuit_controller

#endif // NAV2_REGULATED_PURE_PURSUIT_CONTROLLER__PARAMETER_HANDLER_HPP_
