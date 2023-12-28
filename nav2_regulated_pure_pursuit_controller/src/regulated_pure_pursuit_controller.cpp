// Copyright (c) 2020 Shrijit Singh
// Copyright (c) 2020 Samsung Research America
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
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"
#include "nav2_regulated_pure_pursuit_controller/regulated_pure_pursuit_controller.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"

using std::abs;
using std::hypot;
using std::max;
using std::min;
using namespace nav2_costmap_2d; // NOLINT

namespace nav2_regulated_pure_pursuit_controller
{

void RegulatedPurePursuitController::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
    auto node = parent.lock();
    node_ = parent;

    if (!node)
    {
        throw nav2_core::ControllerException("Unable to lock node!");
    }

    costmap_ros_ = costmap_ros;            // 将传入的 Costmap2DROS 对象赋值给类成员变量 costmap_ros_
    costmap_ = costmap_ros_->getCostmap(); // 从 Costmap2DROS 对象中获取 Costmap2D 并赋值给类成员变量 costmap_
    tf_ = tf;                              // 将传入的 tf2_ros::Buffer 对象赋值给类成员变量 tf_
    plugin_name_ = name;                   // 将传入的控制器名称赋值给类成员变量 plugin_name_
    logger_ = node->get_logger();          // 获取节点的日志记录器并赋值给类成员变量 logger_

    // Handles storage and dynamic configuration of parameters. 处理参数的存储和动态配置。
    // Returns pointer to data current param settings. 返回当前参数设置的数据指针。
    // 通过传入节点、插件名称、日志记录器和 costmap_ 的宽度来创建参数处理器
    param_handler_ = std::make_unique<ParameterHandler>(node, plugin_name_, logger_, costmap_->getSizeInMetersX());

    // 获取当前参数设置的数据指针并赋值给类成员变量 params_
    params_ = param_handler_->getParams();

    // Handles global path transformations 处理全局路径的转换
    // 创建 PathHandler 对象以处理全局路径的转换
    // 通过将参数中的转换容忍时间转换为 duration、传入 tf_ 和 costmap_ros_ 来创建路径处理器
    path_handler_ = std::make_unique<PathHandler>(tf2::durationFromSec(params_->transform_tolerance), tf_, costmap_ros_);

    // Checks for imminent collisions 检查即将发生的碰撞
    // 创建 CollisionChecker 对象以检查即将发生的碰撞
    // 通过传入节点、costmap_ros_ 和 params_ 来创建碰撞检查器
    collision_checker_ = std::make_unique<CollisionChecker>(node, costmap_ros_, params_);

    double control_frequency = 20.0; // 设置默认的控制频率为 20Hz
    goal_dist_tol_ = 0.25;           // reasonable default before first update 设置默认的目标距离容差为 0.25（在第一次更新之前的合理默认值）

    node->get_parameter("controller_frequency", control_frequency); // 从节点参数中获取控制频率参数，如果没有设置，默认为 20Hz
    control_duration_ = 1.0 / control_frequency;                    // 计算控制时间间隔

    global_path_pub_ = node->create_publisher<nav_msgs::msg::Path>("received_global_plan", 1);    // 创建用于发布接收到的全局路径的消息发布者
    carrot_pub_ = node->create_publisher<geometry_msgs::msg::PointStamped>("lookahead_point", 1); // 创建用于发布前瞻点的消息发布者
}

void RegulatedPurePursuitController::cleanup()
{
    // 打印日志，表示正在进行控制器清理操作
    RCLCPP_INFO(
        logger_,
        "Cleaning up controller: %s of type"
        " regulated_pure_pursuit_controller::RegulatedPurePursuitController",
        plugin_name_.c_str());

    global_path_pub_.reset(); // 释放全局路径消息发布者资源
    carrot_pub_.reset();      // 释放前瞻点消息发布者资源
}

// 它将控制器所使用的两个消息发布者（global_path_pub_ 和 carrot_pub_）激活，以便它们可以开始发布消息
void RegulatedPurePursuitController::activate()
{
    // 打印日志，表示正在激活控制器
    RCLCPP_INFO(
        logger_,
        "Activating controller: %s of type "
        "regulated_pure_pursuit_controller::RegulatedPurePursuitController",
        plugin_name_.c_str());
    global_path_pub_->on_activate(); // 激活全局路径消息发布者
    carrot_pub_->on_activate();      // 激活前瞻点消息发布者
}

// 它将控制器所使用的两个 LifecyclePublisher（global_path_pub_ 和 carrot_pub_）进行停用操作，以确保它们在控制器停用状态下不会再发布消息
void RegulatedPurePursuitController::deactivate()
{
    RCLCPP_INFO(
        logger_,
        "Deactivating controller: %s of type "
        "regulated_pure_pursuit_controller::RegulatedPurePursuitController",
        plugin_name_.c_str());
    global_path_pub_->on_deactivate(); // 停用全局路径消息发布者
    carrot_pub_->on_deactivate();      // 停用前瞻点消息发布者
}

std::unique_ptr<geometry_msgs::msg::PointStamped> RegulatedPurePursuitController::createCarrotMsg(
    const geometry_msgs::msg::PoseStamped &carrot_pose)
{
    auto carrot_msg = std::make_unique<geometry_msgs::msg::PointStamped>();
    carrot_msg->header = carrot_pose.header;
    carrot_msg->point.x = carrot_pose.pose.position.x;
    carrot_msg->point.y = carrot_pose.pose.position.y;
    carrot_msg->point.z = 0.01; // publish right over map to stand out 设置 z 坐标为 0.01，以便将前瞻点在地图上进行标识。
    return carrot_msg;
}

double RegulatedPurePursuitController::getLookAheadDistance(
    const geometry_msgs::msg::Twist &speed)
{
    // If using velocity-scaled look ahead distances, find and clamp the dist 如果使用基于速度缩放的前瞻距离，计算并限制前瞻距离
    // Else, use the static look ahead distance  否则，使用静态的前瞻距离
    double lookahead_dist = params_->lookahead_dist;
    if (params_->use_velocity_scaled_lookahead_dist)
    {
        // 计算基于速度缩放的前瞻距离
        lookahead_dist = fabs(speed.linear.x) * params_->lookahead_time;
        // 限制前瞻距离在最小和最大范围内
        lookahead_dist = std::clamp(
            lookahead_dist, params_->min_lookahead_dist, params_->max_lookahead_dist);
    }

    return lookahead_dist;
}

double calculateCurvature(geometry_msgs::msg::Point lookahead_point)
{
    // Find distance^2 to look ahead point (carrot) in robot base frame 在机器人基底框架中计算前瞻点（carrot）的距离平方
    // This is the chord length of the circle 这是圆的弦长
    const double carrot_dist2 =
        (lookahead_point.x * lookahead_point.x) +
        (lookahead_point.y * lookahead_point.y);

    // Find curvature of circle (k = 1 / R) 计算圆的曲率（k = 1 / R）
    if (carrot_dist2 > 0.001)
    {
        return 2.0 * lookahead_point.y / carrot_dist2;
    }
    else
    {
        return 0.0;
    }
}

geometry_msgs::msg::TwistStamped RegulatedPurePursuitController::computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped &pose,
    const geometry_msgs::msg::Twist &speed,
    nav2_core::GoalChecker *goal_checker)
{
    // 使用互斥锁确保对参数的修改是线程安全的
    std::lock_guard<std::mutex> lock_reinit(param_handler_->getMutex());

    // Update for the current goal checker's state  获取当前目标检查器的容差信息
    geometry_msgs::msg::Pose pose_tolerance;
    geometry_msgs::msg::Twist vel_tolerance;
    if (!goal_checker->getTolerances(pose_tolerance, vel_tolerance))
    {
        RCLCPP_WARN(logger_, "Unable to retrieve goal checker's tolerances!");
    }
    else
    {
        goal_dist_tol_ = pose_tolerance.position.x;
    }

    // Transform path to robot base frame  将全局路径转换到机器人基底框架，并发布转换后的路径
    auto transformed_plan = path_handler_->transformGlobalPlan(pose, params_->max_robot_pose_search_dist);
    global_path_pub_->publish(transformed_plan);

    // Find look ahead distance and point on path and publish 计算前瞻距离并找到路径上的前瞻点，并发布相关信息
    double lookahead_dist = getLookAheadDistance(speed);

    // Check for reverse driving 检查是否允许反向行驶
    if (params_->allow_reversing)
    {
        // Cusp check 进行“cusp”检查
        const double dist_to_cusp = findVelocitySignChange(transformed_plan);

        // if the lookahead distance is further than the cusp, use the cusp distance instead 如果前瞻距离比“cusp”距离大，则使用“cusp”距离作为前瞻距离
        if (dist_to_cusp < lookahead_dist)
        {
            lookahead_dist = dist_to_cusp;
        }
    }

    // Get the particular point on the path at the lookahead distance 在路径上找到特定前瞻距离处的前瞻点，并发布该点
    auto carrot_pose = getLookAheadPoint(lookahead_dist, transformed_plan);
    carrot_pub_->publish(createCarrotMsg(carrot_pose));

    double linear_vel, angular_vel; // 初始化线性速度和角速度

    double lookahead_curvature = calculateCurvature(carrot_pose.pose.position); // 计算前瞻点处的曲率

    double regulation_curvature = lookahead_curvature; // 初始化regulation后的曲率

    // 如果使用固定曲率前瞻距离，计算相应前瞻点处的曲率
    if (params_->use_fixed_curvature_lookahead)
    {
        auto curvature_lookahead_pose = getLookAheadPoint(
            params_->curvature_lookahead_dist,
            transformed_plan);
        regulation_curvature = calculateCurvature(curvature_lookahead_pose.pose.position);
    }

    // Setting the velocity direction 设置速度方向
    double sign = 1.0; // 速度方向标志。
    if (params_->allow_reversing)
    {
        sign = carrot_pose.pose.position.x >= 0.0 ? 1.0 : -1.0;
    }

    // 初始化线性速度
    linear_vel = params_->desired_linear_vel;

    // Make sure we're in compliance with basic constraints  // 确保符合基本约束条件
    // 区别在于旋转的目标是机器人是否朝向路径上的特定目标点（shouldRotateToGoalHeading）或仅朝向路径的方向（shouldRotateToPath）。
    // 这两个函数的使用将取决于具体的导航策略和机器人的运动需求。
    double angle_to_heading;
    if (shouldRotateToGoalHeading(carrot_pose))
    {
        // 如果应该朝向目标方向，计算朝向目标的角度
        double angle_to_goal = tf2::getYaw(transformed_plan.poses.back().pose.orientation);
        // 调用旋转函数以调整机器人的朝向
        rotateToHeading(linear_vel, angular_vel, angle_to_goal, speed);
    }
    else if (shouldRotateToPath(carrot_pose, angle_to_heading))
    {
        // 如果应该朝向路径方向，计算朝向路径的角度
        // angle_to_heading 在此处被计算
        // 调用旋转函数以调整机器人的朝向
        rotateToHeading(linear_vel, angular_vel, angle_to_heading, speed);
    }
    else
    {
        // 如果不需要调整朝向，则应用速度约束
        // 包括曲率约束、速度约束以及碰撞检测
        applyConstraints(
            regulation_curvature, speed,
            collision_checker_->costAtPose(pose.pose.position.x, pose.pose.position.y), transformed_plan,
            linear_vel, sign);

        // Apply curvature to angular velocity after constraining linear velocity 在约束线性速度后，应用曲率以计算角速度
        angular_vel = linear_vel * lookahead_curvature;
    }

    // Collision checking on this velocity heading 对这个速度方向进行碰撞检测
    const double &carrot_dist = hypot(carrot_pose.pose.position.x, carrot_pose.pose.position.y);
    if (params_->use_collision_detection &&
        collision_checker_->isCollisionImminent(pose, linear_vel, angular_vel, carrot_dist))
    {
        // 如果开启了碰撞检测且前方有潜在碰撞，抛出异常
        throw nav2_core::NoValidControl("RegulatedPurePursuitController detected collision ahead!");
    }

    // populate and return message  构建并返回一个包含线性和角速度的消息
    geometry_msgs::msg::TwistStamped cmd_vel;
    cmd_vel.header = pose.header;
    cmd_vel.twist.linear.x = linear_vel;
    cmd_vel.twist.angular.z = angular_vel;
    return cmd_vel;
}

//
bool RegulatedPurePursuitController::shouldRotateToPath(
    const geometry_msgs::msg::PoseStamped &carrot_pose, double &angle_to_path)
{
    // Whether we should rotate robot to rough path heading  // 判断是否应该朝向路径方向
    // 计算前瞻点与机器人之间的角度
    angle_to_path = atan2(carrot_pose.pose.position.y, carrot_pose.pose.position.x);
    // 返回是否满足旋转到路径方向的条件
    return params_->use_rotate_to_heading &&
           fabs(angle_to_path) > params_->rotate_to_heading_min_angle;
}

bool RegulatedPurePursuitController::shouldRotateToGoalHeading(
    const geometry_msgs::msg::PoseStamped &carrot_pose)
{
    // Whether we should rotate robot to goal heading 判断是否应该旋转机器人朝向目标朝向
    // 计算前瞻点到目标的距离
    double dist_to_goal = std::hypot(carrot_pose.pose.position.x, carrot_pose.pose.position.y);
    // 返回是否满足旋转到目标朝向的条件
    return params_->use_rotate_to_heading && dist_to_goal < goal_dist_tol_;
}

void RegulatedPurePursuitController::rotateToHeading(
    double &linear_vel, double &angular_vel,
    const double &angle_to_path, const geometry_msgs::msg::Twist &curr_speed)
{
    // Rotate in place using max angular velocity / acceleration possible 使用最大角速度/加速度旋转到位
    linear_vel = 0.0;                                            // 将线性速度设为0，只旋转不移动
    const double sign = angle_to_path > 0.0 ? 1.0 : -1.0;        // 计算旋转方向
    angular_vel = sign * params_->rotate_to_heading_angular_vel; // 根据参数设置角速度

    const double &dt = control_duration_; // 获取控制周期

    // 计算可行的最小和最大角速度
    const double min_feasible_angular_speed = curr_speed.angular.z - params_->max_angular_accel * dt;
    const double max_feasible_angular_speed = curr_speed.angular.z + params_->max_angular_accel * dt;

    // 将角速度限制在可行的范围内
    angular_vel = std::clamp(angular_vel, min_feasible_angular_speed, max_feasible_angular_speed);
}

geometry_msgs::msg::Point RegulatedPurePursuitController::circleSegmentIntersection(
    const geometry_msgs::msg::Point &p1,
    const geometry_msgs::msg::Point &p2,
    double r)
{
    // Formula for intersection of a line with a circle centered at the origin, 计算线段与圆的交点，确保返回的交点在线段之间
    // modified to always return the point that is on the segment between the two points. 这个计算基于线段与圆的交点公式，确保返回的点在给定的线段之间
    // https://mathworld.wolfram.com/Circle-LineIntersection.html
    // This works because the poses are transformed into the robot frame. 这个计算基于将机器人的位姿转换到机器人坐标系中。
    // This can be derived from solving the system of equations of a line and a circle 交点的计算可以从解线段与圆的交点方程组得到，结果相当于二次方程的重写
    // which results in something that is just a reformulation of the quadratic formula.  交互式演示可以在 doc/circle-segment-intersection.ipynb 中找到，也可以在
    // Interactive illustration in doc/circle-segment-intersection.ipynb as well as at
    // https://www.desmos.com/calculator/td5cwbuocd

    // 获取线段的两个端点坐标
    double x1 = p1.x;
    double x2 = p2.x;
    double y1 = p1.y;
    double y2 = p2.y;

    // 计算线段的方向和长度
    double dx = x2 - x1;
    double dy = y2 - y1;
    double dr2 = dx * dx + dy * dy;
    double D = x1 * y2 - x2 * y1;

    // Augmentation to only return point within segment 为了确保交点在线段之间，计算线段两端点到圆心的距离平方
    double d1 = x1 * x1 + y1 * y1;
    double d2 = x2 * x2 + y2 * y2;
    double dd = d2 - d1;

    // 计算交点坐标
    geometry_msgs::msg::Point p;
    double sqrt_term = std::sqrt(r * r * dr2 - D * D);
    p.x = (D * dy + std::copysign(1.0, dd) * dx * sqrt_term) / dr2;
    p.y = (-D * dx + std::copysign(1.0, dd) * dy * sqrt_term) / dr2;
    return p;
}

geometry_msgs::msg::PoseStamped RegulatedPurePursuitController::getLookAheadPoint(
    const double &lookahead_dist,
    const nav_msgs::msg::Path &transformed_plan)
{
    // Find the first pose which is at a distance greater than the lookahead distance // 查找第一个距离大于前瞻距离的姿态点
    auto goal_pose_it = std::find_if(
        transformed_plan.poses.begin(), transformed_plan.poses.end(), [&](const auto &ps)
        { return hypot(ps.pose.position.x, ps.pose.position.y) >= lookahead_dist; });

    // If the no pose is not far enough, take the last pose 如果没有姿态点距离足够远，则选择最后一个姿态点
    if (goal_pose_it == transformed_plan.poses.end())
    {
        goal_pose_it = std::prev(transformed_plan.poses.end()); // 迭代器的前一个
    }
    // 如果启用了插值（use_interpolation 为真），且找到的前瞻点不是路径中的第一个姿态点，那么会进行插值计算，找到机器人与前瞻点构成的线段与圆的交点。
    else if (params_->use_interpolation && goal_pose_it != transformed_plan.poses.begin())
    {
        // Find the point on the line segment between the two poses 找到位于两个姿态点之间的线段上，距离机器人姿态点（原点）恰好为前瞻距离的点
        // that is exactly the lookahead distance away from the robot pose (the origin)
        // This can be found with a closed form for the intersection of a segment and a circle 这可以通过圆与线段的交点的闭合形式来找到
        // Because of the way we did the std::find_if, prev_pose is guaranteed to be inside the circle,  由于我们使用了 std::find_if，prev_pose 确保在圆内
        // and goal_pose is guaranteed to be outside the circle.，    goal_pose 确保在圆外
        auto prev_pose_it = std::prev(goal_pose_it);
        auto point = circleSegmentIntersection(
            prev_pose_it->pose.position,
            goal_pose_it->pose.position, lookahead_dist);
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = prev_pose_it->header.frame_id; // 在路径跟踪中，通常会将前瞻点的帧 ID 设置为与路径中的其他点相同，以确保一致性
        pose.header.stamp = goal_pose_it->header.stamp;       // 而将时间戳设置为与目标点的时间戳相同，可以保持数据的时间一致性，使得在可视化或其他应用中能够正确地关联前瞻点和路径中的其他点。
        pose.pose.position = point;
        return pose;
    }

    return *goal_pose_it;
}

void RegulatedPurePursuitController::applyConstraints(
    const double &curvature, const geometry_msgs::msg::Twist & /*curr_speed*/,
    const double &pose_cost, const nav_msgs::msg::Path &path, double &linear_vel, double &sign)
{
    // 初始化曲率约束速度和成本约束速度
    double curvature_vel = linear_vel, cost_vel = linear_vel;

    // limit the linear velocity by curvature  根据曲率限制线性速度
    if (params_->use_regulated_linear_velocity_scaling)
    {
        curvature_vel = heuristics::curvatureConstraint(
            linear_vel, curvature, params_->regulated_linear_scaling_min_radius);
    }

    // limit the linear velocity by proximity to obstacles 根据避障成本限制线性速度
    if (params_->use_cost_regulated_linear_velocity_scaling)
    {
        cost_vel = heuristics::costConstraint(linear_vel, pose_cost, costmap_ros_, params_);
    }

    // Use the lowest of the 2 constraints, but above the minimum translational speed 选择两个约束中的较小值，但要确保速度不低于最小平移速度
    linear_vel = std::min(cost_vel, curvature_vel);
    linear_vel = std::max(linear_vel, params_->regulated_linear_scaling_min_speed);

    // Apply constraint to reduce speed on approach to the final goal pose 应用靠近目标姿态pose时的速度约束
    linear_vel = heuristics::approachVelocityConstraint(
        linear_vel, path, params_->min_approach_linear_velocity,
        params_->approach_velocity_scaling_dist);

    // Limit linear velocities to be valid 将线性速度限制在有效范围内
    linear_vel = std::clamp(fabs(linear_vel), 0.0, params_->desired_linear_vel);
    linear_vel = sign * linear_vel;
}

void RegulatedPurePursuitController::setPlan(const nav_msgs::msg::Path &path)
{
    path_handler_->setPlan(path);
}

void RegulatedPurePursuitController::setSpeedLimit(
    const double &speed_limit,
    const bool &percentage)
{
    // 使用互斥锁确保对参数的修改是线程安全的
    std::lock_guard<std::mutex> lock_reinit(param_handler_->getMutex());

    // 如果速度限制为特殊值 nav2_costmap_2d::NO_SPEED_LIMIT，恢复默认值
    if (speed_limit == nav2_costmap_2d::NO_SPEED_LIMIT)
    {
        // Restore default value 恢复默认期望线性速度值
        params_->desired_linear_vel = params_->base_desired_linear_vel;
    }
    else
    {
        if (percentage)
        {
            // Speed limit is expressed in % from maximum speed of robot  如果速度限制以百分比形式给出，重新计算期望线性速度
            // 使用给定的速度限制百分比来调整基础期望线性速度
            params_->desired_linear_vel = params_->base_desired_linear_vel * speed_limit / 100.0;
        }
        else
        {
            // Speed limit is expressed in absolute value 如果速度限制是绝对值，直接将期望线性速度设置为给定的速度限制
            params_->desired_linear_vel = speed_limit;
        }
    }
}

double RegulatedPurePursuitController::findVelocitySignChange(
    const nav_msgs::msg::Path &transformed_plan)
{
    // Iterating through the transformed global path to determine the position of the cusp 遍历转换后的全局路径，以确定“cusp”位置
    for (unsigned int pose_id = 1; pose_id < transformed_plan.poses.size() - 1; ++pose_id)
    {
        // We have two vectors for the dot product OA and AB. Determining the vectors. 我们有两个矢量用于点积 OA 和 AB。计算这两个矢量。
        double oa_x = transformed_plan.poses[pose_id].pose.position.x -
                      transformed_plan.poses[pose_id - 1].pose.position.x;
        double oa_y = transformed_plan.poses[pose_id].pose.position.y -
                      transformed_plan.poses[pose_id - 1].pose.position.y;
        double ab_x = transformed_plan.poses[pose_id + 1].pose.position.x -
                      transformed_plan.poses[pose_id].pose.position.x;
        double ab_y = transformed_plan.poses[pose_id + 1].pose.position.y -
                      transformed_plan.poses[pose_id].pose.position.y;

        /* Checking for the existance of cusp, in the path, using the dot product
        and determine it's distance from the robot. If there is no cusp in the path,
        then just determine the distance to the goal location. */
        /* 使用点积检查路径中是否存在“cusp”，
        并确定其距离机器人的距离。如果路径中没有“cusp”，
        则仅计算到目标位置的距离。 */
        if ((oa_x * ab_x) + (oa_y * ab_y) < 0.0)
        {
            // returning the distance if there is a cusp 如果存在“cusp”，返回距离
            // The transformed path is in the robots frame, so robot is at the origin 转换后的路径在机器人坐标系中，因此机器人位于原点
            return hypot(
                transformed_plan.poses[pose_id].pose.position.x,
                transformed_plan.poses[pose_id].pose.position.y);
        }
    }

    return std::numeric_limits<double>::max(); // 如果没有“cusp”，返回一个大的值
}
} // namespace nav2_regulated_pure_pursuit_controller

// Register this controller as a nav2_core plugin
// 这样的声明通常用于实现一种插件系统，例如在ROS中使用的插件系统，其中可以动态加载不同的实现。在这种情况下，
// RegulatedPurePursuitController 类将被声明为一个可以通过插件库导出的类，以便其他部分的代码可以使用它作为 nav2_core::Controller 接口的实现。
PLUGINLIB_EXPORT_CLASS(
    nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController,
    nav2_core::Controller)
