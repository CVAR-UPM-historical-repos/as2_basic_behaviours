#include "takeoff_behaviour.hpp"

TakeOffBehaviour::TakeOffBehaviour() : as2::BasicBehaviour<as2_msgs::action::TakeOff>("TakeOffBehaviour")
{
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        // FIXME: change topic name
        this->generate_global_name("/self_localization/odom"), 10,
        std::bind(&TakeOffBehaviour::odomCb, this, std::placeholders::_1));

    traj_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectoryPoint>(
    this->generate_global_name("motion_reference/trajectory"), 10);
};

rclcpp_action::GoalResponse TakeOffBehaviour::onAccepted(const std::shared_ptr<const as2_msgs::action::TakeOff::Goal> goal)
{
    if ( goal->takeoff_height < 0.0f ) {
        RCLCPP_ERROR(this->get_logger(), "TakeOffBehaviour: Invalid takeoff height");
        return rclcpp_action::GoalResponse::REJECT;
    }

    if ( goal->takeoff_speed < 0.0f ) {
        RCLCPP_ERROR(this->get_logger(), "TakeOffBehaviour: Invalid takeoff speed");
        return rclcpp_action::GoalResponse::REJECT;
    }

    desired_speed_ = (goal->takeoff_speed != 0.0f) ? goal->takeoff_speed : DEFAULT_TAKEOFF_SPEED;
    desired_height_ = (goal->takeoff_height != 0.0f) ? goal->takeoff_height : DEFAULT_TAKEOFF_ALTITUDE;

    RCLCPP_INFO(this->get_logger(), "TakeOffBehaviour: TakeOff with speed %f and height %f", desired_speed_, desired_height_);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
};

rclcpp_action::CancelResponse TakeOffBehaviour::onCancel(const std::shared_ptr<GoalHandleTakeoff> goal_handle)
{
    return rclcpp_action::CancelResponse::ACCEPT;
};

void TakeOffBehaviour::onExecute(const std::shared_ptr<GoalHandleTakeoff> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Executing goal");

    rclcpp::Rate loop_rate(10);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<as2_msgs::action::TakeOff::Feedback>();
    auto result = std::make_shared<as2_msgs::action::TakeOff::Result>();

    rclcpp::Time start_time = this->now();

    // Check if goal is done
    while ((desired_height_ - actual_heigth_) > 0 + TAKEOFF_HEIGHT_THRESHOLD)
    {
        if (goal_handle->is_canceling())
        {
            result->takeoff_success = false;
            goal_handle->canceled(result);
            RCLCPP_INFO(this->get_logger(), "Goal canceled");
            // TODO: change this to hover

            return;
        }

        rclcpp::Time t = this->now();
        trajectory_msgs::msg::JointTrajectoryPoint msg;
        msg.time_from_start.sec = t.seconds() - start_time.seconds();
        msg.time_from_start.nanosec = t.nanoseconds() - start_time.nanoseconds();
        msg.positions = {0.0, 0.0, 0.0, 0.0};
        msg.velocities = {0.0, 0.0, 0.0, desired_speed_};
        msg.accelerations = {0.0, 0.0, 0.0, 0.0};
        traj_pub_->publish(msg);

        // RCLCPP_INFO(this->get_logger(), "Publish feedback");
        feedback->actual_takeoff_height = actual_heigth_;
        feedback->actual_takeoff_speed = actual_z_speed_;
        goal_handle->publish_feedback(feedback);

        loop_rate.sleep();
    }

    result->takeoff_success = true;

    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    // TODO: change this to hover

    trajectory_msgs::msg::JointTrajectoryPoint msg;
    msg.positions = {0.0, 0.0, 0.0, 0.0};
    msg.velocities = {0.0, 0.0, 0.0, 0.0};
    msg.accelerations = {0.0, 0.0, 0.0, 0.0};
    traj_pub_->publish(msg);
};

void TakeOffBehaviour::odomCb(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
    this->odom_msg_ = *(msg.get());
    this->actual_heigth_ = msg->pose.pose.position.z;
    this->actual_z_speed_ = msg->twist.twist.linear.z;
}