#ifndef TAKE_OFF_BEHAVIOUR_HPP 
#define TAKE_OFF_BEHAVIOUR_HPP

#include <memory>
#include <functional>
#include <thread>

#include <as2_basic_behaviour.hpp>
#include "as2_control_command_handlers/position_control.hpp"
#include "as2_control_command_handlers/speed_control.hpp"

#include <aerostack2_msgs/action/take_off.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#define DEFAULT_TAKEOFF_ALTITUDE 1.0 // [m]
#define DEFAULT_TAKEOFF_SPEED 0.4 // [m/s]
#define TAKEOFF_HEIGHT_THRESHOLD 0.1 // [m]

class TakeOffBehaviour : public aerostack2::BasicBehaviour<aerostack2_msgs::action::TakeOff>
{
public:
  using GoalHandleTakeoff = rclcpp_action::ServerGoalHandle<aerostack2_msgs::action::TakeOff>;

  TakeOffBehaviour() : aerostack2::BasicBehaviour<aerostack2_msgs::action::TakeOff>("TakeOffBehaviour")
  {

    speed_controller_ptr_=  std::make_shared<aerostack2::controlCommandsHandlers::SpeedControl>(this);

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
       //FIXME: change topic name
        this->generate_global_name("/self_localization/odom"), 10,
        [this](const nav_msgs::msg::Odometry::ConstSharedPtr msg)
        {
          this->odom_msg_ = *(msg.get());
          this->actual_heigth_ = msg->pose.pose.position.z;
          this->actual_z_speed_ = msg->twist.twist.linear.z;
        });

  };

  rclcpp_action::GoalResponse onAccepted(const std::shared_ptr<const aerostack2_msgs::action::TakeOff::Goal> goal)
  {
    
    
    desired_speed_ = (goal->takeoff_speed!=0.0f)?goal->takeoff_speed: DEFAULT_TAKEOFF_SPEED;
    desired_height_ = (goal->takeoff_height!=0.0f)?goal->takeoff_height: DEFAULT_TAKEOFF_ALTITUDE;

    RCLCPP_INFO(this->get_logger(), "TakeOffBehaviour: TakeOff with speed %f and height %f", desired_speed_, desired_height_);
    
    
    if (speed_controller_ptr_->sendSpeedCommandWithYawAngle(0,0,desired_speed_,odom_msg_.pose.pose.orientation))
    {
      RCLCPP_INFO(this->get_logger(), "Takeoff speed command sent");
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
    else{
      RCLCPP_ERROR(this->get_logger(), "Failed to send takeoff speed command");
      return rclcpp_action::GoalResponse::REJECT;
    }

  };
  rclcpp_action::CancelResponse onCancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<aerostack2_msgs::action::TakeOff>> goal_handle)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  };

  void onExecute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<aerostack2_msgs::action::TakeOff>> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");

    rclcpp::Rate loop_rate(10);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<aerostack2_msgs::action::TakeOff::Feedback>();
    auto result = std::make_shared<aerostack2_msgs::action::TakeOff::Result>();

    // Check if goal is done
    while ((desired_height_ - actual_heigth_) > 0 +TAKEOFF_HEIGHT_THRESHOLD)
    {
      if (goal_handle->is_canceling())
      {
        result->takeoff_success = false;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        //TODO: change this to hover
        speed_controller_ptr_->sendSpeedCommandWithYawAngle(0,0,0,odom_msg_.pose.pose.orientation);
        
        return;
      }

      // RCLCPP_INFO(this->get_logger(), "Publish feedback");
      feedback->actual_takeoff_height = actual_heigth_;
      feedback->actual_takeoff_speed = actual_z_speed_;
      goal_handle->publish_feedback(feedback);

      loop_rate.sleep();
    }

    result->takeoff_success = true;

    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    //TODO: change this to hover
    speed_controller_ptr_->sendSpeedCommandWithYawAngle(0,0,0,odom_msg_.pose.pose.orientation);
  };

private:
  std::atomic<float> actual_heigth_;
  std::atomic<float> actual_z_speed_;
  nav_msgs::msg::Odometry odom_msg_;

  float desired_speed_ = 0.0;
  float desired_height_ = 0.0;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  std::shared_ptr<aerostack2::controlCommandsHandlers::SpeedControl> speed_controller_ptr_;


  
};

#endif // TAKE_OFF_BEHAVIOUR_HPP