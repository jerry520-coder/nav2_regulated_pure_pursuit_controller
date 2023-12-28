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

#include "nav2_regulated_pure_pursuit_controller/collision_checker.hpp"

namespace nav2_regulated_pure_pursuit_controller
{

using namespace nav2_costmap_2d; // NOLINT

CollisionChecker::CollisionChecker(
    rclcpp_lifecycle::LifecycleNode::SharedPtr node,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros,
    Parameters *params)
{
    clock_ = node->get_clock(); // 获取节点的时钟
    costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros_->getCostmap(); // 获取 Costmap2DROS 对象管理的代价地图，并设置成员变量 costmap_
    params_ = params;

    // initialize collision checker and set costmap 创建 FootprintCollisionChecker 对象并初始化
    footprint_collision_checker_ = std::make_unique<nav2_costmap_2d::
                                                        FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>>(costmap_);
    footprint_collision_checker_->setCostmap(costmap_); // 设置 FootprintCollisionChecker 对象的代价地图

    carrot_arc_pub_ = node->create_publisher<nav_msgs::msg::Path>("lookahead_collision_arc", 1); // 创建路径消息发布者，发布碰撞检测的路径弧线
    carrot_arc_pub_->on_activate();                                                              // 激活发布者以准备发布消息
}

// 函数会进行一系列的碰撞检测模拟，并在需要时发布碰撞检测结果的可视化消息。
bool CollisionChecker::isCollisionImminent(
    const geometry_msgs::msg::PoseStamped &robot_pose,
    const double &linear_vel, const double &angular_vel,
    const double &carrot_dist)
{
    // 注意：robot_pose 是在 odom 坐标系下的机器人姿态，而 carrot_pose 是在机器人基座坐标系下的胡萝卜姿态。这是数据传输方式的特殊性质
    // Note(stevemacenski): This may be a bit unusual, but the robot_pose is in
    // odom frame and the carrot_pose is in robot base frame. Just how the data comes to us

    // check current point is OK 检查当前点是否发生碰撞
    if (inCollision(
            robot_pose.pose.position.x, robot_pose.pose.position.y,
            tf2::getYaw(robot_pose.pose.orientation)))
    {
        return true;
    }

    // visualization messages 可视化消息
    nav_msgs::msg::Path arc_pts_msg;
    arc_pts_msg.header.frame_id = costmap_ros_->getGlobalFrameID();
    arc_pts_msg.header.stamp = robot_pose.header.stamp;
    geometry_msgs::msg::PoseStamped pose_msg; // 该消息将用于存储每个位姿点的信息。
    pose_msg.header.frame_id = arc_pts_msg.header.frame_id;
    pose_msg.header.stamp = arc_pts_msg.header.stamp;

    // 如果机器人的线性速度很小（接近于零）且角速度较大，那么机器人可能正在执行旋转操作，
    // 代码会根据代价地图的分辨率和机器人半径来计算旋转需要的时间。否则，如果机器人正在执行普通的路径跟踪操作，
    // 代码会根据代价地图的分辨率和线性速度的绝对值来计算前进需要的时间。
    double projection_time = 0.0;
    // 如果线性速度接近于0且角速度较大
    if (fabs(linear_vel) < 0.01 && fabs(angular_vel) > 0.01)
    {
        // rotating to heading at goal or toward path
        // Equation finds the angular distance required for the largest
        // part of the robot radius to move to another costmap cell:
        // theta_min = 2.0 * sin ((res/2) / r_max)
        // via isosceles triangle r_max-r_max-resolution,
        // dividing by angular_velocity gives us a timestep.
        // 机器人正在旋转以朝向目标或朝向路径
        // 下面的方程用于计算机器人半径的最大部分移动到另一个代价地图单元所需的角距离
        // 该计算通过等腰三角形的概念来实现：r_max - r_max - resolution
        // 将计算结果除以角速度，得到一个时间步长
        // 这样，机器人在角速度的作用下将移动一段距离
        // 这段代码计算的是机器人需要多长时间才能从当前位置旋转到一个新的地图单元
        double max_radius = costmap_ros_->getLayeredCostmap()->getCircumscribedRadius();
        projection_time =
            2.0 * sin((costmap_->getResolution() / 2) / max_radius) / fabs(angular_vel);
    }
    else
    {
        // Normal path tracking 正常路径跟踪
        projection_time = costmap_->getResolution() / fabs(linear_vel); // 投影时间等于代价地图的分辨率除以线性速度的绝对值
    }

    const geometry_msgs::msg::Point &robot_xy = robot_pose.pose.position;
    geometry_msgs::msg::Pose2D curr_pose;
    curr_pose.x = robot_pose.pose.position.x;
    curr_pose.y = robot_pose.pose.position.y;
    // 通过 tf2::getYaw 函数从机器人的姿态中提取机器人的偏航角（角度）
    // 这里假设机器人的姿态是以四元数表示的，通过 getYaw 函数可以将四元数转换为偏航角
    // 偏航角表示了机器人相对于水平方向的旋转角度
    curr_pose.theta = tf2::getYaw(robot_pose.pose.orientation);

    // only forward simulate within time requested 仅在所请求的时间范围内进行前向模拟
    int i = 1;
    while (i * projection_time < params_->max_allowed_time_to_collision_up_to_carrot) // 当前位置和姿态信息进行前向模拟，直到预测的时间超过允许的最大碰撞时间（up to carrot）
    {
        i++;

        // apply velocity at curr_pose over distance // 在当前姿态位置应用线性速度，以前进一定距离
        curr_pose.x += projection_time * (linear_vel * cos(curr_pose.theta));
        curr_pose.y += projection_time * (linear_vel * sin(curr_pose.theta));
        curr_pose.theta += projection_time * angular_vel;

        // check if past carrot pose, where no longer a thoughtfully valid command 、
        // 检查是否已经超过胡萝卜的姿态，即是否已经越过了期望的位置
        // 如果超过了胡萝卜的姿态，不再是一个合理的有效命令
        if (hypot(curr_pose.x - robot_xy.x, curr_pose.y - robot_xy.y) > carrot_dist)
        {
            break;
        }

        // store it for visualization 用于可视化存储
        pose_msg.pose.position.x = curr_pose.x;
        pose_msg.pose.position.y = curr_pose.y;
        pose_msg.pose.position.z = 0.01; // 设置高度信息，用于可视化
        arc_pts_msg.poses.push_back(pose_msg);

        // check for collision at the projected pose 检查在投影姿态处是否发生碰撞
        if (inCollision(curr_pose.x, curr_pose.y, curr_pose.theta))
        {
            // 发布可视化消息，将保存的姿态位置进行可视化显示
            carrot_arc_pub_->publish(arc_pts_msg);
            return true;
        }
    }

    carrot_arc_pub_->publish(arc_pts_msg);

    return false;
}

bool CollisionChecker::inCollision(
    const double &x,
    const double &y,
    const double &theta)
{
    unsigned int mx, my;

    // 将世界坐标转换为 costmap 的栅格坐标
    // 这段代码使用了 RCLCPP_WARN_THROTTLE 宏来生成一个警告日志消息，但是有一个限制条件，即它会限制在一定时间内多次重复显示相同的警告消息。让我们逐步解释它的各个部分：
    // RCLCPP_WARN_THROTTLE: 这是一个宏，用于生成一个警告级别的日志消息，并在一定的时间内限制警告消息的频率。
    // logger_: 这是一个记录器对象，它是用于记录日志的接口，可以使用它来记录不同级别的日志消息。
    // *(clock_): 这是一个指向时钟对象的指针，用于获取当前时间。
    // 30000: 这是限制警告消息频率的时间间隔，单位是毫秒。在这个示例中，警告消息不会频繁地重复显示，至少需要等待 30000 毫秒（30 秒）才会再次显示相同的警告消息。
    if (!costmap_->worldToMap(x, y, mx, my))
    {
        // 如果 costmap 尺寸过小，无法成功检查远处的碰撞情况，给出警告
        RCLCPP_WARN_THROTTLE(
            logger_, *(clock_), 30000,
            "The dimensions of the costmap is too small to successfully check for "
            "collisions as far ahead as requested. Proceed at your own risk, slow the robot, or "
            "increase your costmap size."); // 它指出 costmap 的尺寸太小，无法成功地检查到距离要求那么远的碰撞情况。
        return false;
    }

    // 获取在当前姿态下机器人足迹的碰撞代价
    double footprint_cost = footprint_collision_checker_->footprintCostAtPose(
        x, y, theta, costmap_ros_->getRobotFootprint());
    // 如果足迹代价为 NO_INFORMATION，并且 costmap 正在跟踪未知空间，那么认为没有碰撞
    if (footprint_cost == static_cast<double>(NO_INFORMATION) &&
        costmap_ros_->getLayeredCostmap()->isTrackingUnknown())
    {
        return false;
    }

    // if occupied or unknown and not to traverse unknown space   如果足迹代价为危险障碍物（LETHAL_OBSTACLE）或更高，则认为有碰撞
    return footprint_cost >= static_cast<double>(LETHAL_OBSTACLE);
}

// 用于计算给定位置 (x, y) 处的代价值
double CollisionChecker::costAtPose(const double &x, const double &y)
{
    unsigned int mx, my;

    if (!costmap_->worldToMap(x, y, mx, my))
    {
        RCLCPP_FATAL(
            logger_,
            "The dimensions of the costmap is too small to fully include your robot's footprint, "
            "thusly the robot cannot proceed further");
        throw nav2_core::ControllerException(
            "RegulatedPurePursuitController: Dimensions of the costmap are too small "
            "to encapsulate the robot footprint at current speeds!"); // 它指出 costmap 的尺寸太小，无法完全包含机器人的足迹，因此机器人无法继续前进。
    }

    // 如果映射成功，则获取该格子坐标的代价值，并将其转换为 double 类型返回。
    unsigned char cost = costmap_->getCost(mx, my);
    return static_cast<double>(cost);
}

} // namespace nav2_regulated_pure_pursuit_controller
