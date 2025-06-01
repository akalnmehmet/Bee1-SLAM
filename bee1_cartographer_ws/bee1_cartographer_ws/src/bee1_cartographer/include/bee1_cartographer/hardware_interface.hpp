#ifndef BEE1_CARTOGRAPHER__HARDWARE_INTERFACE_HPP_
#define BEE1_CARTOGRAPHER__HARDWARE_INTERFACE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>

class HardwareInterface : public rclcpp::Node
{
public:
    HardwareInterface();
    ~HardwareInterface() = default;

private:
    void throttleCallback(const std_msgs::msg::Float64::SharedPtr msg);
    void brakeCallback(const std_msgs::msg::Float64::SharedPtr msg);
    void steeringCallback(const std_msgs::msg::Float64::SharedPtr msg);
    void hardwareLoop();
    
    void readAndPublishGPS();
    void readAndPublishIMU();
    void readAndPublishLiDAR();
    
    void sendThrottleCommand(double throttle);
    void sendBrakeCommand(double brake);
    void sendSteeringCommand(double steering);
    
    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_pub_;
    
    // Subscribers
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr throttle_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr brake_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr steering_sub_;
    
    rclcpp::TimerPtr timer_;
    
    // Current vehicle state
    double current_throttle_;
    double current_brake_;
    double current_steering_;
};

#endif  // BEE1_CARTOGRAPHER__HARDWARE_INTERFACE_HPP_