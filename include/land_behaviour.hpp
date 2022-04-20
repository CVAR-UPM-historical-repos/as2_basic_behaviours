#ifndef LAND_BEHAVIOUR_HPP 
#define LAND_BEHAVIOUR_HPP

#include <memory>
#include <functional>
#include <thread>
#include <Eigen/Dense>

#include <as2_basic_behaviour.hpp>
#include "as2_core/names/actions.hpp"
#include "as2_core/names/topics.hpp"
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <as2_msgs/action/land.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include "as2_msgs/msg/controller_control_mode.hpp"
#include "as2_msgs/srv/set_controller_control_mode.hpp"

#define DEFAULT_LAND_ALTITUDE -10.0 // [m]
#define DEFAULT_LAND_SPEED -0.2 // [m/s]

class LandBehaviour : public as2::BasicBehaviour<as2_msgs::action::Land>
{
public:
  using GoalHandleTakeoff = rclcpp_action::ServerGoalHandle<as2_msgs::action::Land>;

  LandBehaviour();

  rclcpp_action::GoalResponse onAccepted(const std::shared_ptr<const as2_msgs::action::Land::Goal> goal);
  rclcpp_action::CancelResponse onCancel(const std::shared_ptr<GoalHandleTakeoff> goal_handle);
  void onExecute(const std::shared_ptr<GoalHandleTakeoff> goal_handle);

private:
  bool checkGoalCondition();
  void odomCb(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
  geometry_msgs::msg::TwistStamped getTwistStamped(Eigen::Vector3d set_speed, double vyaw, double desired_yaw);
  rclcpp::Client<as2_msgs::srv::SetControllerControlMode>::SharedPtr set_control_mode_srv_client_;

private:
  std::atomic<float> actual_heigth_;
  std::atomic<float> actual_z_speed_;
  nav_msgs::msg::Odometry odom_msg_;

  float desired_speed_ = 0.0;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectoryPoint>::SharedPtr traj_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr motion_ref_twist_pub_;
  rclcpp::Time time_;
};

#endif // TAKE_OFF_BEHAVIOUR_HPP