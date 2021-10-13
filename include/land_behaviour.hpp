#ifndef LAND_BEHAVIOUR_HPP 
#define LAND_BEHAVIOUR_HPP

#include <memory>
#include <functional>
#include <thread>

#include <as2_basic_behaviour.hpp>
#include "as2_control_command_handlers/position_control.hpp"
#include "as2_control_command_handlers/speed_control.hpp"

#include <aerostack2_msgs/action/land.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#define DEFAULT_LAND_ALTITUDE -10.0 // [m]
#define DEFAULT_LAND_SPEED -0.2 // [m/s]


class LandBehaviour : public aerostack2::BasicBehaviour<aerostack2_msgs::action::Land>
{
public:
  using GoalHandleTakeoff = rclcpp_action::ServerGoalHandle<aerostack2_msgs::action::Land>;

  LandBehaviour() : aerostack2::BasicBehaviour<aerostack2_msgs::action::Land>("LandBehaviour")
  {

    speed_controller_ptr_=  std::make_shared<aerostack2::controlCommandsHandlers::SpeedControl>(this);

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
       
        this->generate_topic_name("/self_localization/odom"), 10,
        [this](const nav_msgs::msg::Odometry::ConstSharedPtr msg)
        {
          this->odom_msg_ = *(msg.get());
          this->actual_heigth_ = msg->pose.pose.position.z;
          this->actual_z_speed_ = msg->twist.twist.linear.z;
        });

  };

  rclcpp_action::GoalResponse onAccepted(const std::shared_ptr<const aerostack2_msgs::action::Land::Goal> goal)
  {
    
    
    desired_speed_ = (goal->land_speed!=0.0f)?-fabs(goal->land_speed): DEFAULT_LAND_SPEED;
    

    RCLCPP_INFO(this->get_logger(), "LandBehaviour: TakeOff with speed %f and height %f", desired_speed_, desired_height_);
    
    
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
  rclcpp_action::CancelResponse onCancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<aerostack2_msgs::action::Land>> goal_handle)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  };
  
  bool checkGoalCondition(){
    
    if (fabs(actual_z_speed_) < 0.1){
      if ((this->now()-this->time_).seconds() > 1){
        return true;
      }else{
        time_ = this->now();
      }
    }
    
    return false;
  };

  void onExecute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<aerostack2_msgs::action::Land>> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");

    rclcpp::Rate loop_rate(10);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<aerostack2_msgs::action::Land::Feedback>();
    auto result = std::make_shared<aerostack2_msgs::action::Land::Result>();

    time_ = this->now();

    // Check if goal is done
    while (!checkGoalCondition())
    {
      if (goal_handle->is_canceling())
      {
        result->land_success = false;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        //TODO: change this to hover
        speed_controller_ptr_->sendSpeedCommandWithYawAngle(0,0,0,odom_msg_.pose.pose.orientation);
        
        return;
      }

      // RCLCPP_INFO(this->get_logger(), "Publish feedback");
      feedback->actual_land_height = actual_heigth_;
      feedback->actual_land_speed = actual_z_speed_;
      goal_handle->publish_feedback(feedback);

      loop_rate.sleep();
    }

    result->land_success = true;

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
  rclcpp::Time time_; 

  
};

#endif // TAKE_OFF_BEHAVIOUR_HPP