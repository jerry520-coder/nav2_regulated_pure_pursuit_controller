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

#ifndef NAV2_REGULATED_PURE_PURSUIT_CONTROLLER__COLLISION_CHECKER_HPP_
#define NAV2_REGULATED_PURE_PURSUIT_CONTROLLER__COLLISION_CHECKER_HPP_

#include <algorithm>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose2_d.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav2_regulated_pure_pursuit_controller/parameter_handler.hpp"
#include "nav2_util/odometry_utils.hpp"
#include "rclcpp/rclcpp.hpp"

#include "nav2_core/controller_exceptions.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"

namespace nav2_regulated_pure_pursuit_controller
{

/**
 * @class nav2_regulated_pure_pursuit_controller::CollisionChecker
 * @brief Checks for collision based on a RPP control command
 */
class CollisionChecker
{
public:
    /**
     * @brief Constructor for nav2_regulated_pure_pursuit_controller::CollisionChecker
     */
    CollisionChecker(
        rclcpp_lifecycle::LifecycleNode::SharedPtr node,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros, Parameters *params);

    /**
     * @brief Destrructor for nav2_regulated_pure_pursuit_controller::CollisionChecker
     */
    ~CollisionChecker() = default;

    /**
     * @brief Whether collision is imminent 检测是否即将发生碰撞
     * @param robot_pose Pose of robot 机器人的姿态
     * @param carrot_pose Pose of carrot 萝卜点的姿态
     * @param linear_vel linear velocity to forward project 前向投影的线性速度
     * @param angular_vel angular velocity to forward project 前向投影的角速度
     * @param carrot_dist Distance to the carrot for PP 萝卜点距离用于 PP
     * @return Whether collision is imminent 是否即将发生碰撞
     */
    bool isCollisionImminent(
        const geometry_msgs::msg::PoseStamped &,
        const double &, const double &,
        const double &);

    /**
     * @brief checks for collision at projected pose 检测投影姿态是否发生碰撞
     * @param x Pose of pose x
     * @param y Pose of pose y
     * @param theta orientation of Yaw
     * @return Whether in collision
     */
    bool inCollision(
        const double &x,
        const double &y,
        const double &theta);

    /**
     * @brief Cost at a point 获取指定点的代价值
     * @param x Pose of pose x
     * @param y Pose of pose y
     * @return Cost of pose in costmap 代价地图中对应姿态的代价值
     */
    double costAtPose(const double &x, const double &y);

protected:
    rclcpp::Logger logger_{rclcpp::get_logger("RPPCollisionChecker")};
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_; // 指向 Costmap2DROS 类的共享指针，用于管理代价地图
    nav2_costmap_2d::Costmap2D *costmap_;                        // 指向 Costmap2D 类的指针，用于访问代价地图数据

    // 使用 Costmap2D 指针作为模板参数创建一个 FootprintCollisionChecker 的独占指针
    // 用于检查机器人轮廓与代价地图的碰撞
    std::unique_ptr<nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>>
        footprint_collision_checker_;
    Parameters *params_;                                                                        // 指向 Parameters 结构体的指针，用于访问控制器的参数
    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> carrot_arc_pub_; // 指向 LifecyclePublisher 的共享指针，用于发布路径消息
    rclcpp::Clock::SharedPtr clock_;                                                            // 指向 Clock 类的共享指针，用于获取时间信息
};

} // namespace nav2_regulated_pure_pursuit_controller

#endif // NAV2_REGULATED_PURE_PURSUIT_CONTROLLER__COLLISION_CHECKER_HPP_
