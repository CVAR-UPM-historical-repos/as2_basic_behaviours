#include "gotowaypoint_behaviour.hpp"

GoToWaypointBehaviour::GoToWaypointBehaviour() : as2::BasicBehaviour<as2_msgs::action::GoToWaypoint>("GoToWaypointBehaviour")
{
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      this->generate_global_name("/self_localization/odom"), 10,
      std::bind(&GoToWaypointBehaviour::odomCb, this, std::placeholders::_1));

  motion_ref_twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
      this->generate_global_name("/motion_reference/twist"), 10);

  traj_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectoryPoint>(
    this->generate_global_name("motion_reference/trajectory"), 10);
}

rclcpp_action::GoalResponse GoToWaypointBehaviour::onAccepted(const std::shared_ptr<const as2_msgs::action::GoToWaypoint::Goal> goal)
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

  // Assign the goal to the Eigen Vector
  desired_position_ = Eigen::Vector3d(goal->target_pose.position.x, goal->target_pose.position.y, goal->target_pose.position.z);
  desired_speed_ = goal->max_speed;
  ignore_yaw_ = goal->ignore_pose_yaw;
  distance_measured_ = false;

  RCLCPP_INFO(this->get_logger(), "GoToWaypointBehaviour: New goal accepted x: %.2f , y: %.2f , z: %.2f",
              desired_position_[0], desired_position_[1], desired_position_[2]);

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
};

rclcpp_action::CancelResponse GoToWaypointBehaviour::onCancel(const std::shared_ptr<GoalHandleGoToWp> goal_handle)
{
  return rclcpp_action::CancelResponse::ACCEPT;
};

void GoToWaypointBehaviour::onExecute(const std::shared_ptr<GoalHandleGoToWp> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Executing goal");

  rclcpp::Rate loop_rate(10);
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<as2_msgs::action::GoToWaypoint::Feedback>();
  auto result = std::make_shared<as2_msgs::action::GoToWaypoint::Result>();

  double desired_yaw = 0;
  if ( ignore_yaw_ ) {
    pose_mutex_.lock();
    Eigen::Matrix3d rot_mat = actual_q_.toRotationMatrix();
    pose_mutex_.unlock();
    Eigen::Vector3d orientation = rot_mat.eulerAngles(0, 1, 2);
    desired_yaw = orientation[2] + M_PI / 2.0f;
    RCLCPP_INFO(this->get_logger(), "Desired yaw set to %f", desired_yaw);
  }

  time_ = this->now();

  // Check if goal is done
  while (!checkGoalCondition())
  {
    if (goal_handle->is_canceling())
    {
      result->goto_success = false;
      goal_handle->canceled(result);
      // RCLCPP_INFO(this->get_logger(), "Goal cancelled");
      RCLCPP_WARN(this->get_logger(), "Goal cancelled but no new command is sent.");
      // TODO: hover
      return;
    }

    pose_mutex_.lock();
    Eigen::Vector3d set_speed = (desired_position_ - actual_position_);
    pose_mutex_.unlock();

    trajectory_msgs::msg::JointTrajectoryPoint msg;
    rclcpp::Time t = this->now();
    msg.time_from_start.sec = t.seconds() - time_.seconds();
    msg.time_from_start.nanosec = t.nanoseconds() - time_.nanoseconds();
    if ( ignore_yaw_ )
    {
      msg.positions = {0.0, 0.0, 0.0, desired_yaw};
      msg.velocities = {getValidSpeed(set_speed.x()), 
                        getValidSpeed(set_speed.y()), 
                        getValidSpeed(set_speed.z()), 0};
    } else {
      msg.positions = {0.0, 0.0, 0.0, 0.0};
      double vyaw = -atan2f((double)set_speed.x(), (double)set_speed.y()) + M_PI / 2.0f;
      msg.velocities = {getValidSpeed(set_speed.x()), 
                        getValidSpeed(set_speed.y()), 
                        getValidSpeed(set_speed.z()),
                        getValidSpeed(vyaw)};    
    }
    msg.accelerations = {0.0, 0.0, 0.0, 0.0};
    traj_pub_->publish(msg);

    // RCLCPP_INFO(this->get_logger(), "Publish feedback");
    feedback->actual_distance_to_goal = actual_distance_to_goal_;
    feedback->actual_speed = actual_speed_;
    goal_handle->publish_feedback(feedback);

    loop_rate.sleep();
  }

  result->goto_success = true;

  goal_handle->succeed(result);
  RCLCPP_INFO(this->get_logger(), "Goal succeeded");
  // TODO: change this to hover
  // speed_controller_ptr_->sendSpeedCommandWithYawAngle(0,0,0,odom_msg_.pose.pose.orientation);

  trajectory_msgs::msg::JointTrajectoryPoint msg;
  if (ignore_yaw_)
  {
    msg.positions = {0.0, 0.0, 0.0, desired_yaw};
  }
  else
  {
    msg.positions = {0.0, 0.0, 0.0, 0.0};
  }
  msg.velocities = {0.0, 0.0, 0.0, 0.0};
  msg.accelerations = {0.0, 0.0, 0.0, 0.0};
  traj_pub_->publish(msg);
};

void GoToWaypointBehaviour::odomCb(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  this->odom_msg_ = *(msg.get());

  pose_mutex_.lock();
  actual_position_ = {this->odom_msg_.pose.pose.position.x, this->odom_msg_.pose.pose.position.y,
              this->odom_msg_.pose.pose.position.z};
  
  actual_q_ = {this->odom_msg_.pose.pose.orientation.w, this->odom_msg_.pose.pose.orientation.x, 
       this->odom_msg_.pose.pose.orientation.y, this->odom_msg_.pose.pose.orientation.z};
  pose_mutex_.unlock();

  distance_measured_ = true;
  this->actual_distance_to_goal_ = (actual_position_ - desired_position_).norm();
  this->actual_speed_ = Eigen::Vector3d(msg->twist.twist.linear.x,
                                        msg->twist.twist.linear.y,
                                        msg->twist.twist.linear.z).norm();
}

bool GoToWaypointBehaviour::checkGoalCondition()
{
  if (distance_measured_)
  {
    if (fabs(actual_distance_to_goal_) < GOAL_THRESHOLD)
      return true;
  }
  return false;
};

float GoToWaypointBehaviour::getValidSpeed(float speed)
{
  if ( std::abs(speed) > desired_speed_ ) {
    if ( speed < 0.0 ) {
        return -desired_speed_;
    } 
    return desired_speed_;
  }
  return speed;
}
