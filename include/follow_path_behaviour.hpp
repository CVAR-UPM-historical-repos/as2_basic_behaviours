#ifndef FOLLOW_PATH_BEHAVIOUR_HPP 
#define FOLLOW_PATH_BEHAVIOUR_HPP

#include <memory>
#include <functional>
#include <thread>
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
  using GoalHandleTakeoff = rclcpp_action::ServerGoalHandle<as2_msgs::action::FollowPath>;
  
  FollowPathBehaviour() : as2::BasicBehaviour<as2_msgs::action::FollowPath>(as2_names::actions::behaviours::followpath)
  {
    remaining_waypoints_ = -1;  
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        this->generate_global_name(as2_names::topics::self_localization::odom), as2_names::topics::self_localization::qos,
        [this](const nav_msgs::msg::Odometry::ConstSharedPtr msg)
        {
          this->odom_msg_ = *(msg.get());

          Eigen::Vector3d position(this->odom_msg_.pose.pose.position.x,
                                    this->odom_msg_.pose.pose.position.y,
                                    this->odom_msg_.pose.pose.position.z);

          // distance_measured_ = true;
          // this->actual_distance_to_goal_ = (position - desired_position_).norm();
          this->actual_speed_ = Eigen::Vector3d(msg->twist.twist.linear.x,
                                                  msg->twist.twist.linear.y,
                                                  msg->twist.twist.linear.z).norm();
          if (this->remaining_waypoints_ <0 ){
            return;
          }
          
          auto& poses = this->trajectory_waypoints_msg_.poses;
          
          int index = poses.size() - this->remaining_waypoints_;
          if (this->remaining_waypoints_ == 0){
            index = poses.size() - 1;  
          }
          auto next_waypoint = poses[index];

          Eigen::Vector3d next_waypoint_position ( next_waypoint.pose.position.x,
                                                  next_waypoint.pose.position.y,
                                                  next_waypoint.pose.position.z);

          this->actual_distance_to_next_waypoint_ = (position - next_waypoint_position).norm();                                        

          if (((position-next_waypoint_position).norm() < GOAL_THRESHOLD) && this->remaining_waypoints_ > 0){
            this->remaining_waypoints_--;
            next_waypoint_x_= next_waypoint.pose.position.x;
            next_waypoint_y_= next_waypoint.pose.position.y;
            next_waypoint_z_= next_waypoint.pose.position.z;
          }

        });

    traj_waypoints_pub_ = this->create_publisher<as2_msgs::msg::TrajectoryWaypoints>(
        this->generate_global_name(as2_names::topics::motion_reference::wayp), as2_names::topics::motion_reference::qos_wp);

  };

  rclcpp_action::GoalResponse onAccepted(const std::shared_ptr<const as2_msgs::action::FollowPath::Goal> goal)
  {
    trajectory_waypoints_msg_ = goal->trajectory_waypoints;


    if (trajectory_waypoints_msg_.poses.size() == 0)
    {
      RCLCPP_ERROR(this->get_logger(), "Trajectory waypoints are empty");
      remaining_waypoints_ = -1;
      return rclcpp_action::GoalResponse::REJECT;
    }

    remaining_waypoints_ = trajectory_waypoints_msg_.poses.size();

    //Assign the goal to the Eigen Vector
    traj_waypoints_pub_->publish(trajectory_waypoints_msg_);
    RCLCPP_INFO(this->get_logger(), "FollowPathBehaviour: Path command sent");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;

  };
  rclcpp_action::CancelResponse onCancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<as2_msgs::action::FollowPath>> goal_handle)
  {
    remaining_waypoints_ = -1;
    return rclcpp_action::CancelResponse::ACCEPT;
  };

  bool checkGoalCondition(){
    if (remaining_waypoints_ == 0 && actual_speed_ < 0.1){
      return true;
    }
    return false;
  };

  void onExecute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<as2_msgs::action::FollowPath>> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");

    rclcpp::Rate loop_rate(10);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<as2_msgs::action::FollowPath::Feedback>();
    auto result = std::make_shared<as2_msgs::action::FollowPath::Result>();

    time_ = this->now();

    // Check if goal is done
    while (!checkGoalCondition())
    {
      if (goal_handle->is_canceling())
      {
        result->follow_path_success = false;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal cannot be cancelled");
        return;
      }

      // RCLCPP_INFO(this->get_logger(), "Publish feedback");
      feedback->next_waypoint.x=next_waypoint_x_;
      feedback->next_waypoint.y=next_waypoint_y_;
      feedback->next_waypoint.z=next_waypoint_z_;
      feedback->remaining_waypoints = remaining_waypoints_;
      feedback->actual_distance_to_next_waypoint = actual_distance_to_next_waypoint_;
      feedback->actual_speed = actual_speed_;


      goal_handle->publish_feedback(feedback);

      loop_rate.sleep();
    }

    result->follow_path_success = true;
    remaining_waypoints_ = -1;


    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
  };

private:

  std::atomic<float> actual_distance_to_next_waypoint_;
  std::atomic<float> actual_speed_;

  std::atomic<int> remaining_waypoints_;

  std::atomic<float> next_waypoint_x_;
  std::atomic<float> next_waypoint_y_;
  std::atomic<float> next_waypoint_z_;
  
  nav_msgs::msg::Odometry odom_msg_;
  as2_msgs::msg::TrajectoryWaypoints trajectory_waypoints_msg_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<as2_msgs::msg::TrajectoryWaypoints>::SharedPtr traj_waypoints_pub_;

  rclcpp::Time time_; 
  
};

#endif // TAKE_OFF_BEHAVIOUR_HPP