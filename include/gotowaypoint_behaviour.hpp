#ifndef GOTO_BEHAVIOUR_HPP 
#define GOTO_BEHAVIOUR_HPP

#include <memory>
#include <functional>
#include <thread>
#include <Eigen/Dense>

#include <as2_basic_behaviour.hpp>
#include "as2_core/names/actions.hpp"
#include "as2_core/names/topics.hpp"

#include <as2_msgs/action/go_to_waypoint.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#define GOAL_THRESHOLD 0.1 // [m]


class GoToWaypointBehaviour : public as2::BasicBehaviour<as2_msgs::action::GoToWaypoint>
{
public:
  using GoalHandleGoToWp = rclcpp_action::ServerGoalHandle<as2_msgs::action::GoToWaypoint>;

  GoToWaypointBehaviour();

  rclcpp_action::GoalResponse onAccepted(const std::shared_ptr<const as2_msgs::action::GoToWaypoint::Goal> goal);
  rclcpp_action::CancelResponse onCancel(const std::shared_ptr<GoalHandleGoToWp> goal_handle);
  void onExecute(const std::shared_ptr<GoalHandleGoToWp> goal_handle);

private:
  void odomCb(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
  bool checkGoalCondition();
  float getValidSpeed(float speed);

private:
  std::mutex pose_mutex_;
  Eigen::Vector3d actual_position_;
  Eigen::Quaterniond actual_q_;

  std::atomic<float> actual_distance_to_goal_;
  std::atomic<float> actual_speed_;
  std::atomic<bool> distance_measured_;
  
  nav_msgs::msg::Odometry odom_msg_;
  Eigen::Vector3d desired_position_;
  bool ignore_yaw_;

  float desired_speed_ = 0.0;
  float desired_height_ = 0.0;

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr motion_ref_twist_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectoryPoint>::SharedPtr traj_pub_;
  
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Time time_;
};  // GoToWaypointBehaviour class

#endif // GOTO_BEHAVIOUR_HPP