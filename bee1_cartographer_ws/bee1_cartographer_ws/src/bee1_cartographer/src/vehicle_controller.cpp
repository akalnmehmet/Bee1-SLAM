#include "bee1_cartographer/vehicle_controller.hpp"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

VehicleController::VehicleController(const rclcpp::NodeOptions & options)
: Node("vehicle_controller", options),
  target_linear_velocity_(0.0),
  target_angular_velocity_(0.0),
  current_throttle_(0.0),
  current_brake_(0.0),
  current_steering_(0.0),
  emergency_stop_active_(false),
  manual_override_(false),
  max_acceleration_(2.5),
  max_deceleration_(2.5),
  steering_rate_limit_(1.0)
{
    // Declare parameters
    this->declare_parameter("max_speed", MAX_SPEED);
    this->declare_parameter("max_steering_angle", MAX_STEERING_ANGLE);
    this->declare_parameter("max_acceleration", max_acceleration_);
    this->declare_parameter("max_deceleration", max_deceleration_);
    
    // Initialize subscribers
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10,
        std::bind(&VehicleController::cmdVelCallback, this, std::placeholders::_1));
    
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy", 10,
        std::bind(&VehicleController::joyCallback, this, std::placeholders::_1));
    
    emergency_stop_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "emergency_stop", 10,
        std::bind(&VehicleController::emergencyStopCallback, this, std::placeholders::_1));
    
    // Initialize publishers
    throttle_pub_ = this->create_publisher<std_msgs::msg::Float64>("vehicle/throttle", 10);
    brake_pub_ = this->create_publisher<std_msgs::msg::Float64>("vehicle/brake", 10);
    steering_pub_ = this->create_publisher<std_msgs::msg::Float64>("vehicle/steering", 10);
    diagnostics_pub_ = this->create_publisher<std_msgs::msg::String>("vehicle/diagnostics", 10);
    
    // Initialize timers
    control_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),  // 20 Hz control loop
        std::bind(&VehicleController::publishVehicleCommands, this));
    
    diagnostics_timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&VehicleController::publishDiagnostics, this));
    
    RCLCPP_INFO(this->get_logger(), "Vehicle Controller initialized");
}

void VehicleController::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    if (!emergency_stop_active_ && !manual_override_) {
        target_linear_velocity_ = msg->linear.x;
        target_angular_velocity_ = msg->angular.z;
    }
}

void VehicleController::joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    // Implement joystick control for manual override
    if (msg->buttons.size() > 0 && msg->buttons[0]) {  // Button A for manual override
        manual_override_ = true;
        
        if (msg->axes.size() >= 4) {
            target_linear_velocity_ = msg->axes[1] * MAX_SPEED;  // Left stick Y
            target_angular_velocity_ = msg->axes[0] * 1.0;       // Left stick X
        }
    } else {
        manual_override_ = false;
    }
}

void VehicleController::emergencyStopCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    emergency_stop_active_ = msg->data;
    if (emergency_stop_active_) {
        target_linear_velocity_ = 0.0;
        target_angular_velocity_ = 0.0;
        RCLCPP_WARN(this->get_logger(), "EMERGENCY STOP ACTIVATED!");
    }
}

void VehicleController::applySafetyLimits()
{
    // Apply velocity limits
    double max_speed = this->get_parameter("max_speed").as_double();
    target_linear_velocity_ = std::clamp(target_linear_velocity_, -max_speed, max_speed);
    
    double max_steering = this->get_parameter("max_steering_angle").as_double();
    target_angular_velocity_ = std::clamp(target_angular_velocity_, -max_steering, max_steering);
    
    // Emergency stop override
    if (emergency_stop_active_) {
        target_linear_velocity_ = 0.0;
        target_angular_velocity_ = 0.0;
    }
}

void VehicleController::publishVehicleCommands()
{
    applySafetyLimits();
    
    // Convert cmd_vel to vehicle controls
    // Throttle/Brake control
    if (target_linear_velocity_ > 0.0) {
        current_throttle_ = std::min(target_linear_velocity_ / MAX_SPEED, 1.0);
        current_brake_ = 0.0;
    } else if (target_linear_velocity_ < 0.0) {
        current_throttle_ = 0.0;
        current_brake_ = std::min(-target_linear_velocity_ / MAX_SPEED, 1.0);
    } else {
        current_throttle_ = 0.0;
        current_brake_ = 0.2;  // Light braking when stopped
    }
    
    // Steering control (Ackermann)
    if (std::abs(target_linear_velocity_) > 0.1) {
        current_steering_ = std::atan(target_angular_velocity_ * WHEELBASE / target_linear_velocity_);
    } else {
        current_steering_ = target_angular_velocity_;
    }
    current_steering_ = std::clamp(current_steering_, -MAX_STEERING_ANGLE, MAX_STEERING_ANGLE);
    
    // Publish commands
    std_msgs::msg::Float64 throttle_msg;
    throttle_msg.data = current_throttle_;
    throttle_pub_->publish(throttle_msg);
    
    std_msgs::msg::Float64 brake_msg;
    brake_msg.data = current_brake_;
    brake_pub_->publish(brake_msg);
    
    std_msgs::msg::Float64 steering_msg;
    steering_msg.data = current_steering_;
    steering_pub_->publish(steering_msg);
}

void VehicleController::publishDiagnostics()
{
    std_msgs::msg::String diag_msg;
    
    std::stringstream ss;
    ss << "Vehicle Status: ";
    if (emergency_stop_active_) {
        ss << "EMERGENCY_STOP ";
    }
    if (manual_override_) {
        ss << "MANUAL_OVERRIDE ";
    }
    ss << "| Throttle: " << std::fixed << std::setprecision(2) << current_throttle_;
    ss << " | Brake: " << current_brake_;
    ss << " | Steering: " << current_steering_;
    ss << " | Target V: " << target_linear_velocity_;
    ss << " | Target ω: " << target_angular_velocity_;
    
    diag_msg.data = ss.str();
    diagnostics_pub_->publish(diag_msg);
}

// Main function for the node
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VehicleController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}#include "bee1_cartographer/vehicle_controller.hpp"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

VehicleController::VehicleController(const rclcpp::NodeOptions & options)
: Node("vehicle_controller", options),
  target_linear_velocity_(0.0),
  target_angular_velocity_(0.0),
  current_throttle_(0.0),
  current_brake_(0.0),
  current_steering_(0.0),
  emergency_stop_active_(false),
  manual_override_(false),
  max_acceleration_(2.5),
  max_deceleration_(2.5),
  steering_rate_limit_(1.0)
{
    // Declare parameters
    this->declare_parameter("max_speed", MAX_SPEED);
    this->declare_parameter("max_steering_angle", MAX_STEERING_ANGLE);
    this->declare_parameter("max_acceleration", max_acceleration_);
    this->declare_parameter("max_deceleration", max_deceleration_);
    
    // Initialize subscribers
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10,
        std::bind(&VehicleController::cmdVelCallback, this, std::placeholders::_1));
    
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy", 10,
        std::bind(&VehicleController::joyCallback, this, std::placeholders::_1));
    
    emergency_stop_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "emergency_stop", 10,
        std::bind(&VehicleController::emergencyStopCallback, this, std::placeholders::_1));
    
    // Initialize publishers
    throttle_pub_ = this->create_publisher<std_msgs::msg::Float64>("vehicle/throttle", 10);
    brake_pub_ = this->create_publisher<std_msgs::msg::Float64>("vehicle/brake", 10);
    steering_pub_ = this->create_publisher<std_msgs::msg::Float64>("vehicle/steering", 10);
    diagnostics_pub_ = this->create_publisher<std_msgs::msg::String>("vehicle/diagnostics", 10);
    
    // Initialize timers
    control_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),  // 20 Hz control loop
        std::bind(&VehicleController::publishVehicleCommands, this));
    
    diagnostics_timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&VehicleController::publishDiagnostics, this));
    
    RCLCPP_INFO(this->get_logger(), "Vehicle Controller initialized");
}

void VehicleController::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    if (!emergency_stop_active_ && !manual_override_) {
        target_linear_velocity_ = msg->linear.x;
        target_angular_velocity_ = msg->angular.z;
    }
}

void VehicleController::joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    // Implement joystick control for manual override
    if (msg->buttons.size() > 0 && msg->buttons[0]) {  // Button A for manual override
        manual_override_ = true;
        
        if (msg->axes.size() >= 4) {
            target_linear_velocity_ = msg->axes[1] * MAX_SPEED;  // Left stick Y
            target_angular_velocity_ = msg->axes[0] * 1.0;       // Left stick X
        }
    } else {
        manual_override_ = false;
    }
}

void VehicleController::emergencyStopCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    emergency_stop_active_ = msg->data;
    if (emergency_stop_active_) {
        target_linear_velocity_ = 0.0;
        target_angular_velocity_ = 0.0;
        RCLCPP_WARN(this->get_logger(), "EMERGENCY STOP ACTIVATED!");
    }
}

void VehicleController::applySafetyLimits()
{
    // Apply velocity limits
    double max_speed = this->get_parameter("max_speed").as_double();
    target_linear_velocity_ = std::clamp(target_linear_velocity_, -max_speed, max_speed);
    
    double max_steering = this->get_parameter("max_steering_angle").as_double();
    target_angular_velocity_ = std::clamp(target_angular_velocity_, -max_steering, max_steering);
    
    // Emergency stop override
    if (emergency_stop_active_) {
        target_linear_velocity_ = 0.0;
        target_angular_velocity_ = 0.0;
    }
}

void VehicleController::publishVehicleCommands()
{
    applySafetyLimits();
    
    // Convert cmd_vel to vehicle controls
    // Throttle/Brake control
    if (target_linear_velocity_ > 0.0) {
        current_throttle_ = std::min(target_linear_velocity_ / MAX_SPEED, 1.0);
        current_brake_ = 0.0;
    } else if (target_linear_velocity_ < 0.0) {
        current_throttle_ = 0.0;
        current_brake_ = std::min(-target_linear_velocity_ / MAX_SPEED, 1.0);
    } else {
        current_throttle_ = 0.0;
        current_brake_ = 0.2;  // Light braking when stopped
    }
    
    // Steering control (Ackermann)
    if (std::abs(target_linear_velocity_) > 0.1) {
        current_steering_ = std::atan(target_angular_velocity_ * WHEELBASE / target_linear_velocity_);
    } else {
        current_steering_ = target_angular_velocity_;
    }
    current_steering_ = std::clamp(current_steering_, -MAX_STEERING_ANGLE, MAX_STEERING_ANGLE);
    
    // Publish commands
    std_msgs::msg::Float64 throttle_msg;
    throttle_msg.data = current_throttle_;
    throttle_pub_->publish(throttle_msg);
    
    std_msgs::msg::Float64 brake_msg;
    brake_msg.data = current_brake_;
    brake_pub_->publish(brake_msg);
    
    std_msgs::msg::Float64 steering_msg;
    steering_msg.data = current_steering_;
    steering_pub_->publish(steering_msg);
}

void VehicleController::publishDiagnostics()
{
    std_msgs::msg::String diag_msg;
    
    std::stringstream ss;
    ss << "Vehicle Status: ";
    if (emergency_stop_active_) {
        ss << "EMERGENCY_STOP ";
    }
    if (manual_override_) {
        ss << "MANUAL_OVERRIDE ";
    }
    ss << "| Throttle: " << std::fixed << std::setprecision(2) << current_throttle_;
    ss << " | Brake: " << current_brake_;
    ss << " | Steering: " << current_steering_;
    ss << " | Target V: " << target_linear_velocity_;
    ss << " | Target ω: " << target_angular_velocity_;
    
    diag_msg.data = ss.str();
    diagnostics_pub_->publish(diag_msg);
}