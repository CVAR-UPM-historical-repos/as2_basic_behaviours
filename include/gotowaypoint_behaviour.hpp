#ifndef GOTO_BEHAVIOUR_HPP 
#define GOTO_BEHAVIOUR_HPP

#include <memory>
#include <functional>
#include <thread>
#include <Eigen/Dense>

#include <as2_basic_behaviour.hpp>
#include "as2_control_command_handlers/position_control.hpp"
#include "as2_control_command_handlers/speed_control.hpp"

#include <as2_msgs/action/go_to_waypoint.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#define GOAL_THRESHOLD 0.1 // [m]


class GoToWaypointBehaviour : public as2::BasicBehaviour<as2_msgs::action::GoToWaypoint>
{
public:
  using GoalHandleTakeoff = rclcpp_action::ServerGoalHandle<as2_msgs::action::GoToWaypoint>;

  GoToWaypointBehaviour() : as2::BasicBehaviour<as2_msgs::action::GoToWaypoint>("GoToWaypointBehaviour")
  {

    speed_controller_ptr_=  std::make_shared<as2::controlCommandsHandlers::SpeedControl>(this);
    position_controller_ptr_=  std::make_shared<as2::controlCommandsHandlers::PositionControl>(this);

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
       
        this->generate_global_name("/self_localization/odom"), 10,
        [this](const nav_msgs::msg::Odometry::ConstSharedPtr msg)
        {
          this->odom_msg_ = *(msg.get());

          Eigen::Vector3d position(this->odom_msg_.pose.pose.position.x,
                                    this->odom_msg_.pose.pose.position.y,
                                    this->odom_msg_.pose.pose.position.z);

          distance_measured_ = true;
          this->actual_distance_to_goal_ = (position - desired_position_).norm();
          this->actual_speed_ = Eigen::Vector3d(msg->twist.twist.linear.x,
                                                  msg->twist.twist.linear.y,
                                                  msg->twist.twist.linear.z).norm();
        });

  };

  rclcpp_action::GoalResponse onAccepted(const std::shared_ptr<const as2_msgs::action::GoToWaypoint::Goal> goal)
  {

    if ((fabs(goal->target_pose.position.x) +
         fabs(goal->target_pose.position.y) +
         fabs(goal->target_pose.position.z)) == 0.0f)
    {
      RCLCPP_ERROR(this->get_logger(), "Target position is not set");
      return rclcpp_action::GoalResponse::REJECT;
    }
    else if (goal->target_pose.position.z == 0.0f)
    {
      RCLCPP_ERROR(this->get_logger(), "Target height is 0.0 m. Please set a valid target height.");
      return rclcpp_action::GoalResponse::REJECT;
    }
    //Assign the goal to the Eigen Vector
    desired_position_ = Eigen::Vector3d(goal->target_pose.position.x , goal->target_pose.position.y , goal->target_pose.position.z);
    distance_measured_ = false;

    RCLCPP_INFO(this->get_logger(), "GoToWaypointBehaviour: New goal accepted x: %.2f , y: %.2f , z: %.2f", 
                  goal->target_pose.position.x, goal->target_pose.position.y, goal->target_pose.position.z);
    
    bool success = false;
    if (goal->ignore_pose_yaw){
      success = position_controller_ptr_->sendPositionCommandWithYawAngle(goal->target_pose.position.x, 
      goal->target_pose.position.y, goal->target_pose.position.z, this->odom_msg_.pose.pose.orientation);
    }else{
      success =position_controller_ptr_->sendPositionCommandWithYawAngle(goal->target_pose.position.x,
      goal->target_pose.position.y, goal->target_pose.position.z, goal->target_pose.orientation);
    }
    if (success)
    {
      RCLCPP_INFO(this->get_logger(), "GoToWaypointBehaviour: Position command sent");
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
    else{
      RCLCPP_ERROR(this->get_logger(), "GoToWaypointBehaviour: Position command not sent");
      return rclcpp_action::GoalResponse::REJECT;
    }

  };
  rclcpp_action::CancelResponse onCancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<as2_msgs::action::GoToWaypoint>> goal_handle)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  };

  bool checkGoalCondition(){
    if (distance_measured_){
      if (fabs(actual_distance_to_goal_) < GOAL_THRESHOLD) return true;
    }
    return false;
  };

  void onExecute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<as2_msgs::action::GoToWaypoint>> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");

    rclcpp::Rate loop_rate(10);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<as2_msgs::action::GoToWaypoint::Feedback>();
    auto result = std::make_shared<as2_msgs::action::GoToWaypoint::Result>();

    time_ = this->now();

    // Check if goal is done
    while (!checkGoalCondition())
    {
      if (goal_handle->is_canceling())
      {
        result->goto_success = false;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        //TODO: change this to hover
        speed_controller_ptr_->sendSpeedCommandWithYawAngle(0,0,0,odom_msg_.pose.pose.orientation);
        
        return;
      }

      // RCLCPP_INFO(this->get_logger(), "Publish feedback");
      feedback->actual_distance_to_goal = actual_distance_to_goal_;
      feedback->actual_speed = actual_speed_;
      goal_handle->publish_feedback(feedback);

      loop_rate.sleep();
    }

    result->goto_success = true;

    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    //TODO: change this to hover
    //speed_controller_ptr_->sendSpeedCommandWithYawAngle(0,0,0,odom_msg_.pose.pose.orientation);
  };

private:
  std::atomic<float> actual_distance_to_goal_;
  std::atomic<float> actual_speed_;
  std::atomic<bool> distance_measured_;
  
  nav_msgs::msg::Odometry odom_msg_;
  Eigen::Vector3d desired_position_;

  float desired_speed_ = 0.0;
  float desired_height_ = 0.0;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  std::shared_ptr<as2::controlCommandsHandlers::SpeedControl> speed_controller_ptr_;
  std::shared_ptr<as2::controlCommandsHandlers::PositionControl> position_controller_ptr_;
  rclcpp::Time time_; 

  
};

#endif // TAKE_OFF_BEHAVIOUR_HPP