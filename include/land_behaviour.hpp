#ifndef LAND_BEHAVIOUR_HPP 
#define LAND_BEHAVIOUR_HPP

#include <memory>
#include <functional>
#include <thread>

#include <as2_basic_behaviour.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <as2_msgs/action/land.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

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

private:
  std::atomic<float> actual_heigth_;
  std::atomic<float> actual_z_speed_;
  nav_msgs::msg::Odometry odom_msg_;

  float desired_speed_ = 0.0;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectoryPoint>::SharedPtr traj_pub_;
  rclcpp::Time time_;
};

#endif // TAKE_OFF_BEHAVIOUR_HPP