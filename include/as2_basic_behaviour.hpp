// "Copyright [year] <Copyright Owner>"

#ifndef AS2_BASIC_BEHAVIOUR_HPP_
#define AS2_BASIC_BEHAVIOUR_HPP_

#include "as2_core/node.hpp"
#include "as2_core/aerial_platform.hpp"
#include "as2_core/sensor.hpp"
#include "as2_msgs/msg/thrust.hpp"

#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp_action/rclcpp_action.hpp"


#include <memory>
#include <string>
#include <chrono>
#include <cmath>

namespace as2{

template<class MessageT>
class BasicBehaviour: public as2::Node{
public:

  using GoalHandleAction = rclcpp_action::ServerGoalHandle<MessageT>;

  BasicBehaviour(const std::string& name) 
    :Node(name)
    {
    
    this->action_server_ = rclcpp_action::create_server<MessageT>(
      this,
      this->generate_global_name(name),
      std::bind(&BasicBehaviour::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&BasicBehaviour::handleCancel, this, std::placeholders::_1),
      std::bind(&BasicBehaviour::handleAccepted, this, std::placeholders::_1));
    
    };

public:
  virtual rclcpp_action::GoalResponse   onAccepted(const std::shared_ptr<const typename MessageT::Goal> goal) = 0;
  virtual rclcpp_action::CancelResponse onCancel(const std::shared_ptr<GoalHandleAction> goal_handle) = 0;
  virtual void onExecute(const std::shared_ptr<GoalHandleAction> goal_handle) = 0; // return true when finished
  
protected:

private:
  
    // float feedback_freq_;
    typename rclcpp_action::Server<MessageT>::SharedPtr action_server_;

    rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID &uuid,std::shared_ptr<const typename MessageT::Goal> goal){
        RCLCPP_INFO(this->get_logger(), "Received goal request with UUID: %d", uuid);
        // (void)uuid;
        return onAccepted(goal);
    }
    
    rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandleAction> goal_handle){
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        return rclcpp_action::CancelResponse::ACCEPT;
    };
    void handleAccepted(const std::shared_ptr<GoalHandleAction> goal_handle){
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    //TODO: understand why this is necessary
      std::thread{std::bind(&BasicBehaviour::onExecute, this, std::placeholders::_1), goal_handle}.detach();
    };

    // void __execute(const std::shared_ptr<GoalHandleAction> goal_handle) {
    //   RCLCPP_INFO(this->get_logger(), "Executing goal");
    //   rclcpp::Rate loop_rate(feedback_freq_);
    //   while(rclcpp::ok()){
    //     if (goal_handle->is_canceling()){
    //       onCancel(goal_handle);
    //       RCLCPP_INFO(this->get_logger(), "Goal is being cancelled");
    //       return;
    //     }
    //     if (onExecute(goal_handle)){
    //       RCLCPP_INFO(this->get_logger(), "Goal execution finished");
    //       return;
    //     }
    //     loop_rate.sleep(); 
    //   };
    // };

};

} // end namespace as2


#endif