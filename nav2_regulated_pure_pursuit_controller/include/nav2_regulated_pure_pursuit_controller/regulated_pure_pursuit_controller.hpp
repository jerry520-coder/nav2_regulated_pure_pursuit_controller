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

#ifndef NAV2_REGULATED_PURE_PURSUIT_CONTROLLER__REGULATED_PURE_PURSUIT_CONTROLLER_HPP_
#define NAV2_REGULATED_PURE_PURSUIT_CONTROLLER__REGULATED_PURE_PURSUIT_CONTROLLER_HPP_

#include <algorithm>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose2_d.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_regulated_pure_pursuit_controller/collision_checker.hpp"
#include "nav2_regulated_pure_pursuit_controller/parameter_handler.hpp"
#include "nav2_regulated_pure_pursuit_controller/path_handler.hpp"
#include "nav2_regulated_pure_pursuit_controller/regulation_functions.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nav2_regulated_pure_pursuit_controller
{

/**
 * @class nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController
 * @brief Regulated pure pursuit controller plugin  受控纯追踪控制器插件
 */
class RegulatedPurePursuitController : public nav2_core::Controller
{
public:
    /**
     * @brief Constructor for nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController
     */
    RegulatedPurePursuitController() = default;

    /**
     * @brief Destrructor for nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController
     */
    ~RegulatedPurePursuitController() override = default;
    // override关键字说明RegulatedPurePursuitController类重写了它的基类的析构函数，
    // default关键字说明使用编译器默认生成的析构函数。
    // 基类的析构函数必须定义为虚函数，以确保当我们删除一个指向派生类的基类指针时，派生类的析构函数能被正确调用。这是防止资源泄露的重要机制。

    /**
     * @brief Configure controller state machine 配置控制器状态机
     * @param parent WeakPtr to node 节点的 WeakPtr
     * @param name Name of plugin 插件名称
     * @param tf TF buffer  TF 缓冲区
     * @param costmap_ros Costmap2DROS object of environment  环境的 Costmap2DROS 对象
     */
    void configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
        std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

    /**
     * @brief Cleanup controller state machine 清理控制器状态机
     */
    void cleanup() override;

    /**
     * @brief Activate controller state machine 激活控制器状态机
     */
    void activate() override;

    /**
     * @brief Deactivate controller state machine 停用控制器状态机
     */
    void deactivate() override;

    /**
     * @brief Compute the best command given the current pose and velocity, with possible debug information 根据当前姿态和速度计算最佳指令，可能包含调试信息
     *
     * Same as above computeVelocityCommands, but with debug results.
     * If the results pointer is not null, additional information about the twists
     * evaluated will be in results after the call.
     *
     * 与上述 computeVelocityCommands 相同，但包含调试结果。
     * 如果结果指针不为空，在调用后，结果中将包含有关评估的twists的附加信息。
     *
     * @param pose      Current robot pose 当前机器人姿态
     * @param velocity  Current robot velocity 当前机器人速度
     * @param goal_checker   Ptr to the goal checker for this task in case useful in computing commands 用于计算指令的目标检查器的指针
     * @return          Best command 最佳指令
     */
    geometry_msgs::msg::TwistStamped computeVelocityCommands(
        const geometry_msgs::msg::PoseStamped &pose,
        const geometry_msgs::msg::Twist &velocity,
        nav2_core::GoalChecker * /*goal_checker*/) override;

    /**
     * @brief nav2_core setPlan - Sets the global plan 设置全局路径
     * @param path The global plan  全局路径
     */
    void setPlan(const nav_msgs::msg::Path &path) override;

    /**
     * @brief Limits the maximum linear speed of the robot. 限制机器人的最大线速度。
     * @param speed_limit expressed in absolute value (in m/s) 以绝对值表示的速度限制（以 m/s 为单位）
     * or in percentage from maximum robot speed. 或者以最大机器人速度的百分比表示。
     * @param percentage Setting speed limit in percentage if true 如果为真，则以百分比设置速度限制，
     * or in absolute values in false case. 如果为假，则以绝对值设置速度限制。
     */
    void setSpeedLimit(const double &speed_limit, const bool &percentage) override;

protected:
    /**
     * @brief Get lookahead distance 获取前瞻距离
     * @param cmd the current speed to use to compute lookahead point 用于计算前瞻点的当前速度
     * @return lookahead distance 前瞻距离
     */
    double getLookAheadDistance(const geometry_msgs::msg::Twist &);

    /**
     * @brief Creates a PointStamped message for visualization 为可视化创建一个 PointStamped 消息
     * @param carrot_pose Input carrot point as a PoseStamped 输入的carrot点，作为 PoseStamped
     * @return CarrotMsg a carrot point marker, PointStamped 一个carrot点标记，PointStamped
     */
    std::unique_ptr<geometry_msgs::msg::PointStamped> createCarrotMsg(
        const geometry_msgs::msg::PoseStamped &carrot_pose);

    /**
     * @brief Whether robot should rotate to rough path heading  判断机器人是否应该旋转到粗糙路径朝向
     * @param carrot_pose current lookahead point 当前前瞻点
     * @param angle_to_path Angle of robot output relatie to carrot marker 机器人输出与carrot标记的朝向角度
     * @return Whether should rotate to path heading 是否应该旋转到路径朝向
     */
    bool shouldRotateToPath(
        const geometry_msgs::msg::PoseStamped &carrot_pose, double &angle_to_path);

    /**
     * @brief Whether robot should rotate to final goal orientation 判断机器人是否应该旋转到最终目标朝向
     * @param carrot_pose current lookahead point 当前前瞻点
     * @return Whether should rotate to goal heading 是否应该旋转到目标朝向
     */
    bool shouldRotateToGoalHeading(const geometry_msgs::msg::PoseStamped &carrot_pose);

    /**
     * @brief Create a smooth and kinematically smoothed rotation command 创建一个平滑的、在运动学上平滑的旋转指令
     * @param linear_vel linear velocity 线速度
     * @param angular_vel angular velocity 角速度
     * @param angle_to_path Angle of robot output relatie to carrot marker 机器人输出与carrot标记的朝向角度
     * @param curr_speed the current robot speed 当前机器人速度
     */
    void rotateToHeading(
        double &linear_vel, double &angular_vel,
        const double &angle_to_path, const geometry_msgs::msg::Twist &curr_speed);

    /**
     * @brief apply regulation constraints to the system 对系统应用调节约束
     * @param linear_vel robot command linear velocity input 机器人命令的线速度输入
     * @param lookahead_dist optimal lookahead distance 最佳前瞻距离
     * @param curvature curvature of path 路径的曲率
     * @param speed Speed of robot 机器人的速度
     * @param pose_cost cost at this pose 此姿态的代价
     */
    void applyConstraints(
        const double &curvature, const geometry_msgs::msg::Twist &speed,
        const double &pose_cost, const nav_msgs::msg::Path &path,
        double &linear_vel, double &sign);

    /**
     * @brief Find the intersection a circle and a line segment. 查找圆与线段的交点。
     * This assumes the circle is centered at the origin. 这假定圆心位于原点。
     * If no intersection is found, a floating point error will occur.  如果未找到交点，将出现浮点错误。
     * @param p1 first endpoint of line segment 线段的第一个端点
     * @param p2 second endpoint of line segment 线段的第二个端点
     * @param r radius of circle 圆的半径
     * @return point of intersection 交点
     */
    static geometry_msgs::msg::Point circleSegmentIntersection(
        const geometry_msgs::msg::Point &p1,
        const geometry_msgs::msg::Point &p2,
        double r);

    /**
     * @brief Get lookahead point 获取前瞻点
     * @param lookahead_dist Optimal lookahead distance 最佳前瞻距离
     * @param path Current global path 当前全局路径
     * @return Lookahead point 前瞻点
     */
    geometry_msgs::msg::PoseStamped getLookAheadPoint(const double &, const nav_msgs::msg::Path &);

    /**
     * @brief checks for the cusp position 检查拐点位置。 如果存在"cusps"，则返回机器人与该点之间的距离，如果不存在，则返回一个较大的值表示不存在"cusps"
     * @param pose Pose input to determine the cusp position 输入用于确定拐点位置的路径
     * @return robot distance from the cusp 机器人距拐点的距离
     * @note 这个函数的目的是在允许反向行驶时，找到一个较短的前瞻距离，以避免通过拐点。
     */
    double findVelocitySignChange(const nav_msgs::msg::Path &transformed_plan);

    rclcpp_lifecycle::LifecycleNode::WeakPtr node_;                               // rclcpp_lifecycle::LifecycleNode 的弱引用指针，用于与节点进行通信
    std::shared_ptr<tf2_ros::Buffer> tf_;                                         // TF2 缓冲区的共享指针，用于处理变换信息
    std::string plugin_name_;                                                     // 插件名称的字符串，用于标识插件实例
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;                  // Costmap2DROS 的共享指针，用于处理环境的代价地图
    nav2_costmap_2d::Costmap2D *costmap_;                                         // 指向 Costmap2D 对象的指针，用于访问代价地图的数据
    rclcpp::Logger logger_{rclcpp::get_logger("RegulatedPurePursuitController")}; // 日志记录器，用于输出日志信息

    Parameters *params_;      // 参数的指针，用于存储控制器参数
    double goal_dist_tol_;    // 目标距离容差，控制器将视目标点与机器人当前位置的距离小于此值为已到达目标
    double control_duration_; // 控制持续时间，用于计算控制器指令

    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> global_path_pub_;         // 发布全局路径的 LifecyclePublisher 的共享指针
    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PointStamped>> carrot_pub_; // 发布胡萝卜点的 LifecyclePublisher 的共享指针
    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> carrot_arc_pub_;          // 发布胡萝卜弧的 LifecyclePublisher 的共享指针
    std::unique_ptr<nav2_regulated_pure_pursuit_controller::PathHandler> path_handler_;                  // PathHandler 类的唯一指针，用于处理路径信息
    std::unique_ptr<nav2_regulated_pure_pursuit_controller::ParameterHandler> param_handler_;            // ParameterHandler 类的唯一指针，用于处理参数信息
    std::unique_ptr<nav2_regulated_pure_pursuit_controller::CollisionChecker> collision_checker_;        // CollisionChecker 类的唯一指针，用于检查碰撞情况
};

} // namespace nav2_regulated_pure_pursuit_controller

#endif // NAV2_REGULATED_PURE_PURSUIT_CONTROLLER__REGULATED_PURE_PURSUIT_CONTROLLER_HPP_
