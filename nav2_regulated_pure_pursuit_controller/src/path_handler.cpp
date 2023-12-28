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

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "nav2_core/controller_exceptions.hpp"
#include "nav2_regulated_pure_pursuit_controller/path_handler.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"

namespace nav2_regulated_pure_pursuit_controller
{

using nav2_util::geometry_utils::euclidean_distance;

PathHandler::PathHandler(
    tf2::Duration transform_tolerance,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
    : transform_tolerance_(transform_tolerance), tf_(tf), costmap_ros_(costmap_ros)
{
}

// 主要用于获取成本地图的最大范围的一半，通过计算成本地图在 X 和 Y 维度上的尺寸并取最大值，然后将其除以 2.0，以获得成本地图的最大范围的一半。
// 这个值可能在路径规划等任务中使用，以确保规划的路径不会超出成本地图的有效范围。
double PathHandler::getCostmapMaxExtent() const
{
    // 从 Costmap2DROS 对象中获取成本地图的 X 和 Y 维度的尺寸（以米为单位）。 获取 X 和 Y 维度尺寸的最大值
    const double max_costmap_dim_meters = std::max(
        costmap_ros_->getCostmap()->getSizeInMetersX(),
        costmap_ros_->getCostmap()->getSizeInMetersY());
    return max_costmap_dim_meters / 2.0; // 将最大成本地图尺寸除以 2.0，并返回结果
}

// 用于将全局路径中的姿态点转换到机器人的本地坐标系中。
nav_msgs::msg::Path PathHandler::transformGlobalPlan(const geometry_msgs::msg::PoseStamped &pose,
                                                     double max_robot_pose_search_dist)
{
    // 如果全局路径为空，则抛出异常
    if (global_plan_.poses.empty())
    {
        throw nav2_core::InvalidPath("Received plan with zero length");
    }

    // let's get the pose of the robot in the frame of the plan 将机器人的姿态转换到全局路径的坐标系中
    geometry_msgs::msg::PoseStamped robot_pose;
    if (!transformPose(global_plan_.header.frame_id, pose, robot_pose))
    {
        throw nav2_core::ControllerTFError("Unable to transform robot pose into global plan's frame");
    }

    // 找到距离机器人最近的路径点，限制在最大搜索距离内  (返回限定范围路径的最远点的迭代器)
    auto closest_pose_upper_bound = nav2_util::geometry_utils::first_after_integrated_distance(global_plan_.poses.begin(),
                                                                                               global_plan_.poses.end(), max_robot_pose_search_dist);

    // First find the closest pose on the path to the robot 首先找到在路径上最靠近机器人的姿态
    // bounded by when the path turns around (if it does) so we don't get a pose from a later 但不超过路径折返的位置（如果有的话）,以防止获取路径后部的姿态
    // portion of the path
    // lambda 函数计算机器人姿态与当前姿态点之间的欧几里德距离，这样就能找到路径上最近的姿态点，并将其赋值给 transformation_begin迭代器。
    auto transformation_begin = nav2_util::geometry_utils::min_by(global_plan_.poses.begin(), closest_pose_upper_bound,
                                                                  [&robot_pose](const geometry_msgs::msg::PoseStamped &ps)
                                                                  {
                                                                      return euclidean_distance(robot_pose, ps);
                                                                  });

    // We'll discard points on the plan that are outside the local costmap 我们将丢弃路径上在本地代价地图范围之外的点
    // 确定需要转换的路径部分，保证在本地代价地图范围内。
    const double max_costmap_extent = getCostmapMaxExtent();
    auto transformation_end = std::find_if(transformation_begin, global_plan_.poses.end(),
                                           [&](const auto &global_plan_pose)
                                           {
                                               return euclidean_distance(global_plan_pose, robot_pose) > max_costmap_extent;
                                           });

    // Lambda to transform a PoseStamped from global frame to local  用于将全局坐标系中的 PoseStamped 转换为本地坐标系
    // transformGlobalPoseToLocal 函数名
    auto transformGlobalPoseToLocal = [&](const auto &global_plan_pose)
    {
        geometry_msgs::msg::PoseStamped stamped_pose, transformed_pose;
        // 设置转换前姿态点的头部信息
        stamped_pose.header.frame_id = global_plan_.header.frame_id;
        stamped_pose.header.stamp = robot_pose.header.stamp; // 确保了转换后的姿态点与机器人的当前姿态点在时间上保持一致。这对于在不同坐标系之间执行转换、进行数据融合以及确保数据的一致性是很重要的
        stamped_pose.pose = global_plan_pose.pose;
        // 调用 transformPose 函数将全局坐标系姿态点转换为本地坐标系
        if (!transformPose(costmap_ros_->getBaseFrameID(), stamped_pose, transformed_pose))
        {
            // 如果转换失败，抛出异常
            throw nav2_core::ControllerTFError("Unable to transform plan pose into local frame");
        }

        // 将转换后的姿态点的 z 坐标设置为 0
        transformed_pose.pose.position.z = 0.0;
        return transformed_pose;
    };

    // Transform the near part of the global plan into the robot's frame of reference.  将全局路径中靠近机器人的部分转换为机器人的参考坐标系。
    nav_msgs::msg::Path transformed_plan;
    std::transform(
        transformation_begin, transformation_end,
        std::back_inserter(transformed_plan.poses),
        transformGlobalPoseToLocal);
    // 设置转换后路径的头部信息，frame_id 设置为本地代价地图的 frame_id，时间戳设置为机器人当前姿态点的时间戳
    transformed_plan.header.frame_id = costmap_ros_->getBaseFrameID();
    transformed_plan.header.stamp = robot_pose.header.stamp;

    // Remove the portion of the global plan that we've already passed so we don't 移除我们已经经过的全局路径部分，以便在下一次迭代中不再处理它（称为路径修剪）
    // process it on the next iteration (this is called path pruning)
    global_plan_.poses.erase(begin(global_plan_.poses), transformation_begin);

    if (transformed_plan.poses.empty())
    {
        throw nav2_core::InvalidPath("Resulting plan has 0 poses in it.");
    }

    return transformed_plan;
}

bool PathHandler::transformPose(
    const std::string frame,
    const geometry_msgs::msg::PoseStamped &in_pose,
    geometry_msgs::msg::PoseStamped &out_pose) const
{
    // 如果输入位姿已经在目标坐标系中，则无需转换
    if (in_pose.header.frame_id == frame)
    {
        out_pose = in_pose;
        return true;
    }

    try
    {
        // 使用 TF2 库进行坐标变换
        tf_->transform(in_pose, out_pose, frame, transform_tolerance_);

        // 更新输出位姿的坐标系为目标坐标系
        out_pose.header.frame_id = frame;
        return true;
    }
    catch (tf2::TransformException &ex) // 当坐标变换过程中发生异常时，catch 块中的代码会被执行 。ex 是一个引用，意味着它引用了实际的异常对象，而不是其副本
    {
        // 如果发生异常，记录错误信息并返回转换失败
        // 将异常对象 ex 的错误信息（通过 ex.what() 获取）记录到日志中
        RCLCPP_ERROR(logger_, "Exception in transformPose: %s", ex.what());
    }

    return false;
}

} // namespace nav2_regulated_pure_pursuit_controller
