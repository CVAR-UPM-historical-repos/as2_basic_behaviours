#include "follow_path_behaviour.hpp"

FollowPathBehaviour::FollowPathBehaviour() : as2::BasicBehaviour<as2_msgs::action::FollowPath>(as2_names::actions::behaviours::followpath)
{
    waypoints_.clear();
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        this->generate_global_name(as2_names::topics::self_localization::odom), 
        as2_names::topics::self_localization::qos, 
        std::bind(&FollowPathBehaviour::odomCb, this, std::placeholders::_1)
    );

    traj_waypoints_pub_ = this->create_publisher<as2_msgs::msg::TrajectoryWaypoints>(
        this->generate_global_name(as2_names::topics::motion_reference::wayp), as2_names::topics::motion_reference::qos_wp);
};

rclcpp_action::GoalResponse FollowPathBehaviour::onAccepted(const std::shared_ptr<const as2_msgs::action::FollowPath::Goal> goal)
{
    as2_msgs::msg::TrajectoryWaypoints trajectory_waypoints = goal->trajectory_waypoints;
    if (trajectory_waypoints.poses.size() == 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Trajectory waypoints are empty");
        waypoints_.clear();
        return rclcpp_action::GoalResponse::REJECT;
    }

    // Populate Waypoints queue
    for(auto it = trajectory_waypoints.poses.begin(); it != trajectory_waypoints.poses.end(); ++it ) {
        waypoints_.push_back(Eigen::Vector3d(it->pose.position.x, it->pose.position.y, it->pose.position.z));
    }

    // Assign the goal to the Eigen Vector
    traj_waypoints_pub_->publish(trajectory_waypoints);
    RCLCPP_INFO(this->get_logger(), "FollowPathBehaviour: Path command sent");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
};

rclcpp_action::CancelResponse FollowPathBehaviour::onCancel(const std::shared_ptr<GoalHandleFollowPath> goal_handle)
{
    // TODO: since follow path is done by traj_gen + controllor, cancel has to be also handle by them
    waypoints_.clear();
    return rclcpp_action::CancelResponse::ACCEPT;
};

void FollowPathBehaviour::onExecute(const std::shared_ptr<GoalHandleFollowPath> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Executing goal");

    rclcpp::Rate loop_rate(10);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<as2_msgs::action::FollowPath::Feedback>();
    auto result = std::make_shared<as2_msgs::action::FollowPath::Result>();

    // Fixme get next (first) from queue
    Eigen::Vector3d next_wayp = waypoints_.front();
    waypoints_.pop_front();

    Eigen::Vector3d position(this->current_pose_x_,
                            this->current_pose_y_,
                            this->current_pose_z_);
    float distance_to_next_waypoint = (position - next_wayp).norm();

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

        position = Eigen::Vector3d(this->current_pose_x_, this->current_pose_y_, this->current_pose_z_);
        distance_to_next_waypoint = (position - next_wayp).norm();

        if ( (distance_to_next_waypoint < GOAL_THRESHOLD) && !waypoints_.empty() )
        {
            next_wayp = waypoints_.front();
            waypoints_.pop_front();
        }

        // RCLCPP_INFO(this->get_logger(), "Publish feedback");
        feedback->next_waypoint.x = next_wayp[0];
        feedback->next_waypoint.y = next_wayp[1];
        feedback->next_waypoint.z = next_wayp[2];
        feedback->remaining_waypoints = waypoints_.size();
        feedback->actual_distance_to_next_waypoint = distance_to_next_waypoint;
        feedback->actual_speed = actual_speed_;

        goal_handle->publish_feedback(feedback);

        loop_rate.sleep();
    }

    result->follow_path_success = true;
    waypoints_.clear();

    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
};

bool FollowPathBehaviour::checkGoalCondition()
{
    if (waypoints_.empty() && actual_speed_ < 0.1)
    {
        return true;
    }
    return false;
};

void FollowPathBehaviour::odomCb(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
    nav_msgs::msg::Odometry odom_msg = *(msg.get());

    this->current_pose_x_ = odom_msg.pose.pose.position.x;
    this->current_pose_y_ = odom_msg.pose.pose.position.y;
    this->current_pose_z_ = odom_msg.pose.pose.position.z;

    this->actual_speed_ = Eigen::Vector3d(msg->twist.twist.linear.x,
                                          msg->twist.twist.linear.y,
                                          msg->twist.twist.linear.z).norm();
}