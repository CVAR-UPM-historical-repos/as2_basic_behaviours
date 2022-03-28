#ifndef TAKE_OFF_BEHAVIOUR_HPP
#define TAKE_OFF_BEHAVIOUR_HPP

#include <memory>
#include <functional>
#include <thread>

#include <as2_basic_behaviour.hpp>
#include "as2_control_command_handlers/position_control.hpp"
#include "as2_control_command_handlers/speed_control.hpp"

#include <as2_msgs/action/take_off.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#define DEFAULT_TAKEOFF_ALTITUDE 1.0 // [m]
#define DEFAULT_TAKEOFF_SPEED 0.4    // [m/s]
#define TAKEOFF_HEIGHT_THRESHOLD 0.1 // [m]

class TakeOffBehaviour : public as2::BasicBehaviour<as2_msgs::action::TakeOff>
{
public:
  using GoalHandleTakeoff = rclcpp_action::ServerGoalHandle<as2_msgs::action::TakeOff>;

  TakeOffBehaviour();

  rclcpp_action::GoalResponse onAccepted(const std::shared_ptr<const as2_msgs::action::TakeOff::Goal> goal);
  rclcpp_action::CancelResponse onCancel(const std::shared_ptr<GoalHandleTakeoff> goal_handle);
  void onExecute(const std::shared_ptr<GoalHandleTakeoff> goal_handle);

private:
  void odomCb(const nav_msgs::msg::Odometry::ConstSharedPtr msg);

private:
  std::atomic<float> actual_heigth_;
  std::atomic<float> actual_z_speed_;
  nav_msgs::msg::Odometry odom_msg_;

  float desired_speed_ = 0.0;
  float desired_height_ = 0.0;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectoryPoint>::SharedPtr traj_pub_;
};

#endif // TAKE_OFF_BEHAVIOUR_HPP