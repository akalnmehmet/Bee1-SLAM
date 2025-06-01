#ifndef BEE1_CARTOGRAPHER__MISSION_EXECUTOR_HPP_
#define BEE1_CARTOGRAPHER__MISSION_EXECUTOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/action/navigate_through_poses.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/string.hpp>
#include <vector>
#include <memory>
#include <string>

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using NavigateThroughPoses = nav2_msgs::action::NavigateThroughPoses;
using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;
using GoalHandleNavigateThroughPoses = rclcpp_action::ClientGoalHandle<NavigateThroughPoses>;

enum class MissionState {
    IDLE,
    NAVIGATING,
    COMPLETED,
    FAILED
};

class MissionExecutor : public rclcpp::Node
{
public:
    explicit MissionExecutor(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    ~MissionExecutor() = default;

private:
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
    void startMission();
    void navigateToNextWaypoint();
    void goalResponseCallback(const GoalHandleNavigateToPose::SharedPtr & goal_handle);
    void feedbackCallback(const GoalHandleNavigateToPose::SharedPtr,
                         const std::shared_ptr<const NavigateToPose::Feedback> feedback);
    void resultCallback(const GoalHandleNavigateToPose::WrappedResult & result);
    void publishMissionStatus();
    
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscriber_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
    rclcpp::TimerPtr timer_;
    
    std::vector<geometry_msgs::msg::PoseStamped> waypoints_;
    size_t current_waypoint_index_;
    MissionState mission_state_;
    
    bool mission_started_;
    bool auto_start_;
};

#endif  // BEE1_CARTOGRAPHER__MISSION_EXECUTOR_HPP_