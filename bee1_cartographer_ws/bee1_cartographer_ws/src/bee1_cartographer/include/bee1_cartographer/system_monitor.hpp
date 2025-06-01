#ifndef BEE1_CARTOGRAPHER__SYSTEM_MONITOR_HPP_
#define BEE1_CARTOGRAPHER__SYSTEM_MONITOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <string>
#include <memory>
#include <cmath>

class SystemMonitor : public rclcpp::Node
{
public:
    SystemMonitor();
    ~SystemMonitor() = default;

private:
    void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void missionStatusCallback(const std_msgs::msg::String::SharedPtr msg);
    void publishDiagnostics();
    
    // Publishers
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr system_status_pub_;
    
    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mission_status_sub_;
    
    rclcpp::TimerPtr timer_;
    
    // Status variables
    std::string gps_status_;
    std::string imu_status_;
    std::string mission_status_;
    double gps_latitude_;
    double gps_longitude_;
    double gps_altitude_;
    double current_linear_vel_;
    double current_angular_vel_;
    
    rclcpp::Time last_gps_time_;
    rclcpp::Time last_imu_time_;
    rclcpp::Time last_cmd_vel_time_;
};

#endif  // BEE1_CARTOGRAPHER__SYSTEM_MONITOR_HPP_