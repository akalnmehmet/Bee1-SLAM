#include "bee1_cartographer/mission_executor.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

MissionExecutor::MissionExecutor(const rclcpp::NodeOptions & options)
: Node("mission_executor", options),
  current_waypoint_index_(0),
  mission_state_(MissionState::IDLE),
  mission_started_(false)
{
    // Declare parameters
    this->declare_parameter("auto_start", false);
    auto_start_ = this->get_parameter("auto_start").as_bool();
    
    // Initialize subscribers
    path_subscriber_ = this->create_subscription<nav_msgs::msg::Path>(
        "mission_waypoints", 10,
        std::bind(&MissionExecutor::pathCallback, this, std::placeholders::_1));
    
    // Initialize publishers
    status_publisher_ = this->create_publisher<std_msgs::msg::String>(
        "mission_status", 10);
    
    // Initialize navigation action client
    nav_client_ = rclcpp_action::create_client<NavigateToPose>(
        this, "navigate_to_pose");
    
    // Status timer
    timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&MissionExecutor::publishMissionStatus, this));
    
    RCLCPP_INFO(this->get_logger(), "Mission Executor initialized. Auto-start: %s", 
                auto_start_ ? "true" : "false");
}

void MissionExecutor::pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
    waypoints_.clear();
    for (const auto& pose : msg->poses) {
        waypoints_.push_back(pose);
    }
    
    RCLCPP_INFO(this->get_logger(), "Received mission path with %zu waypoints", 
                waypoints_.size());
    
    if (auto_start_ && !mission_started_ && !waypoints_.empty()) {
        startMission();
    }
}

void MissionExecutor::startMission()
{
    if (waypoints_.empty()) {
        RCLCPP_WARN(this->get_logger(), "No waypoints available to start mission");
        return;
    }
    
    if (!nav_client_->wait_for_action_server(std::chrono::seconds(10))) {
        RCLCPP_ERROR(this->get_logger(), "Navigation action server not available");
        mission_state_ = MissionState::FAILED;
        return;
    }
    
    mission_started_ = true;
    current_waypoint_index_ = 0;
    mission_state_ = MissionState::NAVIGATING;
    
    RCLCPP_INFO(this->get_logger(), "Starting mission with %zu waypoints", waypoints_.size());
    navigateToNextWaypoint();
}

void MissionExecutor::navigateToNextWaypoint()
{
    if (current_waypoint_index_ >= waypoints_.size()) {
        RCLCPP_INFO(this->get_logger(), "Mission completed successfully!");
        mission_state_ = MissionState::COMPLETED;
        return;
    }
    
    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose = waypoints_[current_waypoint_index_];
    
    RCLCPP_INFO(this->get_logger(), "Navigating to waypoint %zu: (%.2f, %.2f)",
                current_waypoint_index_,
                goal_msg.pose.pose.position.x,
                goal_msg.pose.pose.position.y);
    
    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.goal_response_callback = 
        std::bind(&MissionExecutor::goalResponseCallback, this, std::placeholders::_1);
    send_goal_options.feedback_callback = 
        std::bind(&MissionExecutor::feedbackCallback, this, 
                  std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback = 
        std::bind(&MissionExecutor::resultCallback, this, std::placeholders::_1);
    
    nav_client_->async_send_goal(goal_msg, send_goal_options);
}

void MissionExecutor::goalResponseCallback(const GoalHandleNavigateToPose::SharedPtr & goal_handle)
{
    if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        mission_state_ = MissionState::FAILED;
    } else {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
}

void MissionExecutor::feedbackCallback(
    const GoalHandleNavigateToPose::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback)
{
    auto current_pose = feedback->current_pose;
    RCLCPP_DEBUG(this->get_logger(), 
                "Current position: (%.2f, %.2f)", 
                current_pose.pose.position.x,
                current_pose.pose.position.y);
}

void MissionExecutor::resultCallback(const GoalHandleNavigateToPose::WrappedResult & result)
{
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Waypoint %zu reached successfully", 
                        current_waypoint_index_);
            current_waypoint_index_++;
            navigateToNextWaypoint();
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Navigation aborted");
            mission_state_ = MissionState::FAILED;
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(this->get_logger(), "Navigation canceled");
            mission_state_ = MissionState::FAILED;
            break;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown navigation result code");
            mission_state_ = MissionState::FAILED;
            break;
    }
}

void MissionExecutor::publishMissionStatus()
{
    std_msgs::msg::String status_msg;
    
    switch (mission_state_) {
        case MissionState::IDLE:
            status_msg.data = "IDLE - Waiting for mission start";
            break;
        case MissionState::NAVIGATING:
            status_msg.data = "NAVIGATING - Waypoint " + 
                              std::to_string(current_waypoint_index_ + 1) + 
                              "/" + std::to_string(waypoints_.size());
            break;
        case MissionState::COMPLETED:
            status_msg.data = "COMPLETED - Mission finished successfully";
            break;
        case MissionState::FAILED:
            status_msg.data = "FAILED - Mission aborted";
            break;
    }
    
    status_publisher_->publish(status_msg);
}

// Main function for the node
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MissionExecutor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}