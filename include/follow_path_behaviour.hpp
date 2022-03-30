#ifndef FOLLOW_PATH_BEHAVIOUR_HPP 
#define FOLLOW_PATH_BEHAVIOUR_HPP

#include <memory>
#include <deque>          // std::deque
#include <Eigen/Dense>

#include <as2_basic_behaviour.hpp>
#include "as2_core/names/actions.hpp"
#include "as2_core/names/topics.hpp"

#include <as2_msgs/action/follow_path.hpp>
#include <as2_msgs/msg/trajectory_waypoints.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#define GOAL_THRESHOLD 0.5 // [m]


class FollowPathBehaviour : public as2::BasicBehaviour<as2_msgs::action::FollowPath>
{
public:
  using GoalHandleFollowPath = rclcpp_action::ServerGoalHandle<as2_msgs::action::FollowPath>;
  
  FollowPathBehaviour();

  rclcpp_action::GoalResponse onAccepted(const std::shared_ptr<const as2_msgs::action::FollowPath::Goal> goal);
  rclcpp_action::CancelResponse onCancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<as2_msgs::action::FollowPath>> goal_handle);

  void onExecute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<as2_msgs::action::FollowPath>> goal_handle);
private:
  bool checkGoalCondition();
  void odomCb(const nav_msgs::msg::Odometry::ConstSharedPtr msg);

private:
  std::atomic<float> current_pose_x_;
  std::atomic<float> current_pose_y_;
  std::atomic<float> current_pose_z_;

  std::atomic<float> actual_speed_;

  std::deque<Eigen::Vector3d> waypoints_;
  
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<as2_msgs::msg::TrajectoryWaypoints>::SharedPtr traj_waypoints_pub_;  
}; // FollowPathBehaviour class

#endif // FOLLOW_PATH_BEHAVIOUR_HPP