#ifndef BEE1_CARTOGRAPHER__VEHICLE_CONTROLLER_HPP_
#define BEE1_CARTOGRAPHER__VEHICLE_CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <memory>
#include <algorithm>
#include <cmath>

class VehicleController : public rclcpp::Node
{
public:
    explicit VehicleController(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    ~VehicleController() = default;

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);
    void emergencyStopCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void publishVehicleCommands();
    void applySafetyLimits();
    void publishDiagnostics();
    
    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_stop_sub_;
    
    // Publishers
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr throttle_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr brake_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr steering_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr diagnostics_pub_;
    
    rclcpp::TimerPtr control_timer_;
    rclcpp::TimerPtr diagnostics_timer_;
    
    // Vehicle parameters
    static constexpr double MAX_SPEED = 15.0;  // m/s (55 km/h)
    static constexpr double MAX_STEERING_ANGLE = 0.5;  // radians
    static constexpr double WHEELBASE = 1.86;  // meters
    
    // Control variables
    double target_linear_velocity_;
    double target_angular_velocity_;
    double current_throttle_;
    double current_brake_;
    double current_steering_;
    
    bool emergency_stop_active_;
    bool manual_override_;
    
    // Safety parameters
    double max_acceleration_;
    double max_deceleration_;
    double steering_rate_limit_;
};

#endif  // BEE1_CARTOGRAPHER__VEHICLE_CONTROLLER_HPP_