#include "land_behaviour.hpp"

LandBehaviour::LandBehaviour() : as2::BasicBehaviour<as2_msgs::action::Land>(as2_names::actions::behaviours::land)
{
    traj_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectoryPoint>(
        this->generate_global_name(as2_names::topics::motion_reference::trajectory), 
        as2_names::topics::motion_reference::qos
    );

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        this->generate_global_name(as2_names::topics::self_localization::odom), 
        as2_names::topics::self_localization::qos,
        std::bind(&LandBehaviour::odomCb, this, std::placeholders::_1)
    );
};

rclcpp_action::GoalResponse LandBehaviour::onAccepted(const std::shared_ptr<const as2_msgs::action::Land::Goal> goal)
{
    desired_speed_ = (goal->land_speed != 0.0f) ? -fabs(goal->land_speed) : DEFAULT_LAND_SPEED;
    RCLCPP_INFO(this->get_logger(), "LandBehaviour: Land with speed %f", desired_speed_);

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
};

rclcpp_action::CancelResponse LandBehaviour::onCancel(const std::shared_ptr<GoalHandleTakeoff> goal_handle)
{
    return rclcpp_action::CancelResponse::ACCEPT;
};

void LandBehaviour::onExecute(const std::shared_ptr<GoalHandleTakeoff> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Executing goal");

    rclcpp::Rate loop_rate(10);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<as2_msgs::action::Land::Feedback>();
    auto result = std::make_shared<as2_msgs::action::Land::Result>();

    time_ = this->now();

    // Check if goal is done
    while (!checkGoalCondition())
    {
        if (goal_handle->is_canceling())
        {
            result->land_success = false;
            goal_handle->canceled(result);
            RCLCPP_INFO(this->get_logger(), "Goal canceled");
            // TODO: change this to hover

            return;
        }

        rclcpp::Time t = this->now();
        trajectory_msgs::msg::JointTrajectoryPoint msg;
        msg.time_from_start.sec = t.seconds() - time_.seconds();
        msg.time_from_start.nanosec = t.nanoseconds() - time_.nanoseconds();
        msg.positions = {0.0, 0.0, 0.0, 0.0};
        msg.velocities = {0.0, 0.0, 0.0, desired_speed_};
        msg.accelerations = {0.0, 0.0, 0.0, 0.0};
        traj_pub_->publish(msg);

        // RCLCPP_INFO(this->get_logger(), "Publish feedback");
        feedback->actual_land_height = actual_heigth_;
        feedback->actual_land_speed = actual_z_speed_;
        goal_handle->publish_feedback(feedback);

        loop_rate.sleep();
    }

    result->land_success = true;

    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    // TODO: change this to hover

    trajectory_msgs::msg::JointTrajectoryPoint msg;
    msg.positions = {0.0, 0.0, 0.0, 0.0};
    msg.velocities = {0.0, 0.0, 0.0, 0.0};
    msg.accelerations = {0.0, 0.0, 0.0, 0.0};
    traj_pub_->publish(msg);
};

bool LandBehaviour::checkGoalCondition()
{
    if (fabs(actual_z_speed_) < 0.1)
    {
        if ((this->now() - this->time_).seconds() > 1)
        {
            return true;
        }
        else
        {
            time_ = this->now();
        }
    }
    return false;
};

void LandBehaviour::odomCb(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
    this->odom_msg_ = *(msg.get());
    this->actual_heigth_ = msg->pose.pose.position.z;
    this->actual_z_speed_ = msg->twist.twist.linear.z;
}