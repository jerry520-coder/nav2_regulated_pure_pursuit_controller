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

#ifndef NAV2_REGULATED_PURE_PURSUIT_CONTROLLER__PATH_HANDLER_HPP_
#define NAV2_REGULATED_PURE_PURSUIT_CONTROLLER__PATH_HANDLER_HPP_

#include <algorithm>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose2_d.hpp"
#include "nav2_core/controller_exceptions.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/odometry_utils.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nav2_regulated_pure_pursuit_controller
{

/**
 * @class nav2_regulated_pure_pursuit_controller::PathHandler
 * @brief Handles input paths to transform them to local frames required 处理输入的路径，将其转换到所需的本地坐标系中
 */
class PathHandler
{
public:
    /**
     * @brief Constructor for nav2_regulated_pure_pursuit_controller::PathHandler
     */
    PathHandler(
        tf2::Duration transform_tolerance,
        std::shared_ptr<tf2_ros::Buffer> tf,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros);

    /**
     * @brief Destrructor for nav2_regulated_pure_pursuit_controller::PathHandler
     */
    ~PathHandler() = default;

    /**
     * @brief Transforms global plan into same frame as pose and clips poses ineligible for lookaheadPoint
     * Points ineligible to be selected as a lookahead point if they are any of the following:
     * - Outside the local_costmap (collision avoidance cannot be assured)
     * 将全局路径转换到与pose相同的坐标系中，并裁剪不符合前视点要求的pose
     * 不符合前视点选取条件的点包括以下情况：
     * - 在局部代价地图local_costmap外部（无法保证避碰）
     * @param pose pose to transform 要进行转换的姿态pose
     * @param max_robot_pose_search_dist Distance to search for matching nearest path point 搜索匹配最近路径点的最大距离
     * @return Path in new frame 在新坐标系中的路径
     */
    nav_msgs::msg::Path transformGlobalPlan(
        const geometry_msgs::msg::PoseStamped &pose,
        double max_robot_pose_search_dist);

    /**
     * @brief Transform a pose to another frame. 将pose转换到另一个坐标系
     * @param frame Frame ID to transform to 要转换到的坐标系的帧 ID
     * @param in_pose Pose input to transform 要转换的输入姿态
     * @param out_pose transformed output 转换后的输出姿态
     * @return bool if successful 转换是否成功
     */
    bool transformPose(
        const std::string frame,
        const geometry_msgs::msg::PoseStamped &in_pose,
        geometry_msgs::msg::PoseStamped &out_pose) const;

    void setPlan(const nav_msgs::msg::Path &path) { global_plan_ = path; }

    nav_msgs::msg::Path getPlan() { return global_plan_; }

protected:
    /**
     * Get the greatest extent of the costmap in meters from the center. 获取代价地图的最大范围（以米为单位）从中心到边缘的距离
     * @return max of distance from center in meters to edge of costmap 以米为单位的代价地图最大范围距离
     */
    double getCostmapMaxExtent() const; // 获取成本地图的最大范围

    rclcpp::Logger logger_{rclcpp::get_logger("RPPPathHandler")}; // 创建一个日志记录器
    tf2::Duration transform_tolerance_;                           // 时间间隔，用于容忍坐标变换的时间延迟
    std::shared_ptr<tf2_ros::Buffer> tf_;                         // 共享的 TF2 缓冲区，用于管理坐标变换
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;  // 共享的 Costmap2DROS 对象，用于管理成本地图
    nav_msgs::msg::Path global_plan_;                             // 存储全局路径的消息对象
};

} // namespace nav2_regulated_pure_pursuit_controller

#endif // NAV2_REGULATED_PURE_PURSUIT_CONTROLLER__PATH_HANDLER_HPP_
