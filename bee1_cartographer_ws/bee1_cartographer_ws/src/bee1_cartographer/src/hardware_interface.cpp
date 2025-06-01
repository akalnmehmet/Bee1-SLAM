#include "bee1_cartographer/hardware_interface.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/twist.hpp>

HardwareInterface::HardwareInterface() : Node("hardware_interface"),
    current_throttle_(0.0), current_brake_(0.0), current_steering_(0.0)
{
    // GPS Publisher
    gps_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("gps/fix", 10);
    
    // IMU Publisher  
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);
    
    // LiDAR Publisher
    lidar_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("velodyne_points", 10);
    
    // Vehicle control subscribers
    throttle_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "vehicle/throttle", 10,
        std::bind(&HardwareInterface::throttleCallback, this, std::placeholders::_1));
        
    brake_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "vehicle/brake", 10,
        std::bind(&HardwareInterface::brakeCallback, this, std::placeholders::_1));
        
    steering_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "vehicle/steering", 10,
        std::bind(&HardwareInterface::steeringCallback, this, std::placeholders::_1));
    
    // Hardware communication timer
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),  // 100 Hz
        std::bind(&HardwareInterface::hardwareLoop, this));
        
    RCLCPP_INFO(this->get_logger(), "Hardware Interface initialized");
}

void HardwareInterface::throttleCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
    // Send throttle command to vehicle hardware
    current_throttle_ = msg->data;
    sendThrottleCommand(current_throttle_);
}

void HardwareInterface::brakeCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
    // Send brake command to vehicle hardware
    current_brake_ = msg->data;
    sendBrakeCommand(current_brake_);
}

void HardwareInterface::steeringCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
    // Send steering command to vehicle hardware
    current_steering_ = msg->data;
    sendSteeringCommand(current_steering_);
}

void HardwareInterface::hardwareLoop()
{
    // Read sensor data and publish
    readAndPublishGPS();
    readAndPublishIMU();
    readAndPublishLiDAR();
}

void HardwareInterface::readAndPublishGPS()
{
    // Read GPS data from hardware
    auto gps_msg = sensor_msgs::msg::NavSatFix();
    gps_msg.header.stamp = this->get_clock()->now();
    gps_msg.header.frame_id = "gps_link";
    
    // TODO: Read from actual UBLOX GPS
    gps_msg.latitude = 40.7903314;  // Dummy data
    gps_msg.longitude = 29.50896659;
    gps_msg.altitude = 100.0;
    gps_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
    
    gps_pub_->publish(gps_msg);
}

void HardwareInterface::readAndPublishIMU()
{
    // Read IMU data from hardware
    auto imu_msg = sensor_msgs::msg::Imu();
    imu_msg.header.stamp = this->get_clock()->now();
    imu_msg.header.frame_id = "imu_link";
    
    // TODO: Read from actual XSENS IMU
    // Dummy data for now
    imu_msg.orientation.w = 1.0;
    imu_msg.angular_velocity.z = 0.0;
    imu_msg.linear_acceleration.x = 0.0;
    
    imu_pub_->publish(imu_msg);
}

void HardwareInterface::readAndPublishLiDAR()
{
    // LiDAR data is handled by velodyne driver
    // This is just a placeholder for custom processing
}

void HardwareInterface::sendThrottleCommand(double throttle)
{
    // TODO: Send to actual vehicle CAN bus
    RCLCPP_DEBUG(this->get_logger(), "Throttle: %.2f", throttle);
}

void HardwareInterface::sendBrakeCommand(double brake)
{
    // TODO: Send to actual vehicle CAN bus
    RCLCPP_DEBUG(this->get_logger(), "Brake: %.2f", brake);
}

void HardwareInterface::sendSteeringCommand(double steering)
{
    // TODO: Send to actual vehicle CAN bus
    RCLCPP_DEBUG(this->get_logger(), "Steering: %.2f", steering);
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HardwareInterface>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}