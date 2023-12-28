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

#ifndef NAV2_REGULATED_PURE_PURSUIT_CONTROLLER__REGULATION_FUNCTIONS_HPP_
#define NAV2_REGULATED_PURE_PURSUIT_CONTROLLER__REGULATION_FUNCTIONS_HPP_

#include <algorithm>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_regulated_pure_pursuit_controller/parameter_handler.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nav2_regulated_pure_pursuit_controller
{

namespace heuristics
{

/**
 * @brief apply curvature constraint regulation on the linear velocity  在线性速度上应用曲率约束【曲率启发式】
 * @param raw_linear_velocity Raw linear velocity desired 原始期望的线性速度
 * @param curvature Curvature of the current command to follow the path 当前路径跟随命令的曲率
 * @param min_radius Minimum path radius to apply the heuristic 最小路径半径，以应用该启发式规则
 * @return Velocity after applying the curvature constraint 应用曲率约束后的速度
 */
inline double curvatureConstraint(
    const double raw_linear_vel, const double curvature, const double min_radius)
{
    const double radius = fabs(1.0 / curvature); // 计算路径的曲率半径
    if (radius < min_radius)                     // 检查曲率半径是否小于最小半径
    {
        // 根据曲率半径的差异来调整线性速度
        return raw_linear_vel * (1.0 - (fabs(radius - min_radius) / min_radius));
    }
    else
    {
        // 曲率半径在合适范围内，无需调整线性速度
        return raw_linear_vel;
    }
}

/**
 * @brief apply cost constraint regulation on the linear velocity 在线性速度上应用代价约束【接近障碍物启发式】
 * @param raw_linear_velocity Raw linear velocity desired 原始期望的线性速度
 * @param pose_cost Cost at the robot pose  机器人姿态的代价
 * @param costmap_ros Costmap object to query 用于查询的代价图对象
 * @param params Parameters 参数
 * @return Velocity after applying the cost constraint 应用代价约束后的速度
 */
inline double costConstraint(
    const double raw_linear_vel,
    const double pose_cost,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros,
    Parameters *params)
{
    using namespace nav2_costmap_2d; // NOLINT  引入nav2_costmap_2d命名空间

    // 检查pose_cost是否有效，不是NO_INFORMATION或FREE_SPACE。 这是因为代价地图中不同代价值对应不同的地形类型，只有在不是未知信息或自由空间时，才需要考虑代价约束。
    if (pose_cost != static_cast<double>(NO_INFORMATION) &&
        pose_cost != static_cast<double>(FREE_SPACE))
    {
        // 获取代价地图的内切半径
        const double &inscribed_radius = costmap_ros->getLayeredCostmap()->getInscribedRadius(); // 代价地图包含不同层级的代价值，而内切半径则用于计算机器人周围的代价。

        // 计算最小距离到障碍物，使用代价因子进行缩放
        // inflation_cost_scaling_factor 表示障碍物膨胀因子。障碍物的代价将被膨胀以考虑机器人周围的安全空间。这是一个可调节的参数，用于平衡避障和移动效率之间的关系。
        // inscribed_radius  这是机器人的内切半径。内切半径表示机器人中心到边界的最短距离，即机器人在不发生碰撞的情况下的最小尺寸。
        // log(pose_cost) 这个操作将代价值从线性尺度转换为对数尺度，可以用于一些数值计算和调整的目的
        // log(253.0f)  这是一个常数，表示一个预定义的代价值。在这里，它的对数被用作一个参考点。
        // 它首先通过将代价值的对数与预定义常数的对数相减，计算出机器人到障碍物的相对距离。然后，这个相对距离通过乘以膨胀因子进行缩放，以考虑机器人的内切半径和障碍物膨胀。
        const double min_distance_to_obstacle =
            (params->inflation_cost_scaling_factor * inscribed_radius - log(pose_cost) + log(253.0f)) /
            params->inflation_cost_scaling_factor;

        // 检查最小距离是否小于代价约束距离
        if (min_distance_to_obstacle < params->cost_scaling_dist)
        {
            // 计算调整因子，并使用此因子调整原始的线性速度
            return raw_linear_vel *
                   (params->cost_scaling_gain * min_distance_to_obstacle / params->cost_scaling_dist);
        }
    }

    // 如果不满足代价约束条件，直接返回原始线性速度
    return raw_linear_vel;
}

/**
 * @brief Compute the scale factor to apply for linear velocity regulation on approach to goal 这个函数用于计算靠近目标时的速度缩放因子
 * @param transformed_path Path to use to calculate distances to goal 用于计算到目标的距离的路径
 * @param approach_velocity_scaling_dist Minimum distance away to which to apply the heuristic 开始应用启发式规则的最小距离
 * @return A scale from 0.0-1.0 of the distance to goal scaled by minimum distance 距离到目标的比例尺从0.0到1.0，缩放到最小距离
 * @note 这个因子用于将速度逐渐减小，以平稳地接近目标点
 */
inline double approachVelocityScalingFactor(
    const nav_msgs::msg::Path &transformed_path,
    const double approach_velocity_scaling_dist)
{
    using namespace nav2_util::geometry_utils; // NOLINT

    // Waiting to apply the threshold based on integrated distance ensures we don't
    // erroneously apply approach scaling on curvy（弯曲的） paths that are contained in a large local costmap.
    // 等待根据积分距离应用阈值，以确保我们不会错误地在包含在large local costmap中的曲线路径上应用 接近缩放。
    const double remaining_distance = calculate_path_length(transformed_path); // 通过 calculate_path_length 函数计算路径的剩余长度（积分距离）
    if (remaining_distance < approach_velocity_scaling_dist)                   // 然后根据这个积分距离和指定的缩放距离来确定是否应用缩放
    {
        // 如果剩余距离小于缩放距离，则计算机器人当前位置到路径终点的欧几里得距离
        auto &last = transformed_path.poses.back();
        // Here we will use a regular euclidean distance from the robot frame (origin) 这里我们将使用机器人坐标系（原点）的常规欧几里得距离
        // to get smooth scaling, regardless of path density. 以获得平滑的缩放，而不考虑路径密度。
        return std::hypot(last.pose.position.x, last.pose.position.y) / approach_velocity_scaling_dist; // 并将其除以缩放距离，以得到速度缩放因子
    }
    else
    {
        return 1.0; // 如果剩余距离大于等于缩放距离，则返回缩放因子为 1.0，表示不需要速度缩放。
    }
}

/**
 * @brief Velocity on approach to goal heuristic regulation term 靠近目标时的速度启发式规则约束项
 * @param constrained_linear_vel Linear velocity already constrained by heuristics 已经受到规则约束的线性速度
 * @param path The path plan in the robot base frame coordinates 机器人基底坐标系中的路径规划
 * @param min_approach_velocity Minimum velocity to use on approach to goal 靠近目标时的最小速度
 * @param approach_velocity_scaling_dist Distance away from goal to start applying this heuristic  开始应用这个启发式规则的距离
 * @return Velocity after regulation via approach to goal slow-down 应用靠近目标减速后的速度
 * @note 它首先计算靠近目标时的速度缩放因子，然后将原始速度乘以该因子，得到约束后的速度
 */
inline double approachVelocityConstraint(
    const double constrained_linear_vel,
    const nav_msgs::msg::Path &path,
    const double min_approach_velocity,
    const double approach_velocity_scaling_dist)
{
    // 计算速度缩放因子
    double velocity_scaling = approachVelocityScalingFactor(path, approach_velocity_scaling_dist);

    // 计算约束后的速度
    double approach_vel = constrained_linear_vel * velocity_scaling;

    // 如果约束后的速度小于最小靠近速度，使用最小靠近速度
    if (approach_vel < min_approach_velocity)
    {
        approach_vel = min_approach_velocity;
    }

    // 返回约束后的速度与原始速度的较小值
    return std::min(constrained_linear_vel, approach_vel);
}

} // namespace heuristics

} // namespace nav2_regulated_pure_pursuit_controller

#endif // NAV2_REGULATED_PURE_PURSUIT_CONTROLLER__REGULATION_FUNCTIONS_HPP_
