#include "takeoff_behaviour.hpp"

TakeOffBehaviour::TakeOffBehaviour() : as2::BasicBehaviour<as2_msgs::action::TakeOff>(as2_names::actions::behaviours::takeoff)
{
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        this->generate_global_name(as2_names::topics::self_localization::odom), as2_names::topics::self_localization::qos,
        std::bind(&TakeOffBehaviour::odomCb, this, std::placeholders::_1));

    traj_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectoryPoint>(
    this->generate_global_name(as2_names::topics::motion_reference::trajectory), as2_names::topics::motion_reference::qos);
    
    motion_ref_twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
        this->generate_global_name(as2_names::topics::motion_reference::twist), as2_names::topics::motion_reference::qos);

    // set_control_mode_srv_client_ = this->create_client<as2_msgs::srv::SetControllerControlMode>(this->generate_global_name(as2_names::services::motion_reference::setcontrolmode));
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

    // auto request = std::make_shared<as2_msgs::srv::SetControllerControlMode::Request>();
    // request->control_mode.mode = as2_msgs::msg::ControllerControlMode::SPEED;

    // while (!set_control_mode_srv_client_->wait_for_service(std::chrono::seconds(1))) {
    //     // RCLCPP_INFO(node_ptr_->get_logger(), "waiting for service ok");
    //     if (!rclcpp::ok()) {
    //         RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
    //         return;
    //     }
    //     RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
    // }

    // auto srv_result = set_control_mode_srv_client_->async_send_request(request);


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

        RCLCPP_INFO(this->get_logger(), "Publishing twist point");
        Eigen::Vector3d set_speed = Eigen::Vector3d::Zero();
        set_speed.z() = desired_speed_;
        RCLCPP_INFO(this->get_logger(), "Getting twist point");
        geometry_msgs::msg::TwistStamped msg_twist = TakeOffBehaviour::getTwistStamped(set_speed, 0.0, 0.0);
        RCLCPP_INFO(this->get_logger(), "Publishing twist point %f %f %f", msg_twist.twist.linear.x, msg_twist.twist.linear.y, msg_twist.twist.linear.z);
        motion_ref_twist_pub_->publish(msg_twist);
        RCLCPP_INFO(this->get_logger(), "Twist point published");

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

    Eigen::Vector3d set_speed = Eigen::Vector3d::Zero();
    motion_ref_twist_pub_->publish(TakeOffBehaviour::getTwistStamped(set_speed, 0.0, 0.0));

};

void TakeOffBehaviour::odomCb(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
    this->odom_msg_ = *(msg.get());
    this->actual_heigth_ = msg->pose.pose.position.z;
    this->actual_z_speed_ = msg->twist.twist.linear.z;
}

geometry_msgs::msg::TwistStamped TakeOffBehaviour::getTwistStamped(Eigen::Vector3d set_speed, double vyaw, double desired_yaw)
{
    geometry_msgs::msg::TwistStamped msg;
    msg.header.stamp = rclcpp::Time(0);
    msg.twist.linear.x = set_speed.x();
    msg.twist.linear.y = set_speed.y();
    msg.twist.linear.z = set_speed.z();
    msg.twist.angular.y = desired_yaw;
    msg.twist.angular.z = vyaw;
    return msg;
}