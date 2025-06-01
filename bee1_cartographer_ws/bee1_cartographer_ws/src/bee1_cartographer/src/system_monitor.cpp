#include "bee1_cartographer/system_monitor.hpp"
#include <sstream>
#include <chrono>
#include <iomanip>

SystemMonitor::SystemMonitor() : Node("system_monitor"),
    gps_status_("UNKNOWN"), imu_status_("UNKNOWN"), mission_status_("UNKNOWN"),
    gps_latitude_(0.0), gps_longitude_(0.0), gps_altitude_(0.0),
    current_linear_vel_(0.0), current_angular_vel_(0.0)
{
    // Publishers
    diagnostics_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
        "diagnostics", 10);
    system_status_pub_ = this->create_publisher<std_msgs::msg::String>(
        "system_status", 10);
    
    // Subscribers for monitoring
    gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "gps/fix", 10,
        std::bind(&SystemMonitor::gpsCallback, this, std::placeholders::_1));
        
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "imu/data", 10,
        std::bind(&SystemMonitor::imuCallback, this, std::placeholders::_1));
        
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10,
        std::bind(&SystemMonitor::cmdVelCallback, this, std::placeholders::_1));
        
    mission_status_sub_ = this->create_subscription<std_msgs::msg::String>(
        "mission_status", 10,
        std::bind(&SystemMonitor::missionStatusCallback, this, std::placeholders::_1));
    
    // Timer for periodic monitoring
    timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&SystemMonitor::publishDiagnostics, this));
        
    // Initialize timestamps
    last_gps_time_ = this->get_clock()->now();
    last_imu_time_ = this->get_clock()->now();
    last_cmd_vel_time_ = this->get_clock()->now();
    
    RCLCPP_INFO(this->get_logger(), "System Monitor initialized");
}

void SystemMonitor::gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
    last_gps_time_ = this->get_clock()->now();
    
    switch(msg->status.status) {
        case sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX:
            gps_status_ = "NO_FIX";
            break;
        case sensor_msgs::msg::NavSatStatus::STATUS_FIX:
            gps_status_ = "FIX";
            break;
        case sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX:
            gps_status_ = "SBAS_FIX";
            break;
        case sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX:
            gps_status_ = "GBAS_FIX";
            break;
        default:
            gps_status_ = "UNKNOWN";
    }
    
    gps_latitude_ = msg->latitude;
    gps_longitude_ = msg->longitude;
    gps_altitude_ = msg->altitude;
}

void SystemMonitor::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    last_imu_time_ = this->get_clock()->now();
    imu_status_ = "ACTIVE";
    
    // Check for reasonable IMU data
    double accel_magnitude = sqrt(
        msg->linear_acceleration.x * msg->linear_acceleration.x +
        msg->linear_acceleration.y * msg->linear_acceleration.y +
        msg->linear_acceleration.z * msg->linear_acceleration.z
    );
    
    if (accel_magnitude < 5.0 || accel_magnitude > 15.0) {
        imu_status_ = "WARNING";
    }
}

void SystemMonitor::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    last_cmd_vel_time_ = this->get_clock()->now();
    current_linear_vel_ = msg->linear.x;
    current_angular_vel_ = msg->angular.z;
}

void SystemMonitor::missionStatusCallback(const std_msgs::msg::String::SharedPtr msg)
{
    mission_status_ = msg->data;
}

void SystemMonitor::publishDiagnostics()
{
    auto now = this->get_clock()->now();
    auto diagnostics_msg = diagnostic_msgs::msg::DiagnosticArray();
    diagnostics_msg.header.stamp = now;
    
    // GPS Status
    auto gps_diag = diagnostic_msgs::msg::DiagnosticStatus();
    gps_diag.name = "GPS";
    gps_diag.hardware_id = "UBLOX";
    
    auto gps_age = (now - last_gps_time_).seconds();
    if (gps_age > 2.0) {
        gps_diag.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        gps_diag.message = "GPS data timeout";
    } else if (gps_status_ == "NO_FIX") {
        gps_diag.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        gps_diag.message = "GPS no fix";
    } else {
        gps_diag.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
        gps_diag.message = "GPS " + gps_status_;
    }
    
    diagnostics_msg.status.push_back(gps_diag);
    
    // IMU Status
    auto imu_diag = diagnostic_msgs::msg::DiagnosticStatus();
    imu_diag.name = "IMU";
    imu_diag.hardware_id = "XSENS_MTi680";
    
    auto imu_age = (now - last_imu_time_).seconds();
    if (imu_age > 1.0) {
        imu_diag.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        imu_diag.message = "IMU data timeout";
    } else {
        imu_diag.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
        imu_diag.message = "IMU " + imu_status_;
    }
    
    diagnostics_msg.status.push_back(imu_diag);
    
    // Vehicle Status
    auto vehicle_diag = diagnostic_msgs::msg::DiagnosticStatus();
    vehicle_diag.name = "Vehicle";
    vehicle_diag.hardware_id = "Bee1";
    
    if (std::abs(current_linear_vel_) > 16.0) {  // Over speed limit
        vehicle_diag.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        vehicle_diag.message = "Speed over limit";
    } else {
        vehicle_diag.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
        vehicle_diag.message = "Vehicle nominal";
    }
    
    diagnostics_msg.status.push_back(vehicle_diag);
    
    // Publish diagnostics
    diagnostics_pub_->publish(diagnostics_msg);
    
    // Publish system status summary
    auto status_msg = std_msgs::msg::String();
    std::stringstream ss;
    ss << "System Status - ";
    ss << "GPS: " << gps_status_ << " | ";
    ss << "IMU: " << imu_status_ << " | ";
    ss << "Mission: " << mission_status_ << " | ";
    ss << "Speed: " << std::fixed << std::setprecision(1) << current_linear_vel_ << " m/s";
    
    status_msg.data = ss.str();
    system_status_pub_->publish(status_msg);
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SystemMonitor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
                // Initialize status
        gps_status_ = "UNKNOWN";
        imu_status_ = "UNKNOWN";
        mission_status_ = "UNKNOWN";
        last_gps_time_ = this->get_clock()->now();
        last_imu_time_ = this->get_clock()->now();
        last_cmd_vel_time_ = this->get_clock()->now();
        
        RCLCPP_INFO(this->get_logger(), "System Monitor initialized");
    }

private:
    void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        last_gps_time_ = this->get_clock()->now();
        
        switch(msg->status.status) {
            case sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX:
                gps_status_ = "NO_FIX";
                break;
            case sensor_msgs::msg::NavSatStatus::STATUS_FIX:
                gps_status_ = "FIX";
                break;
            case sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX:
                gps_status_ = "SBAS_FIX";
                break;
            case sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX:
                gps_status_ = "GBAS_FIX";
                break;
            default:
                gps_status_ = "UNKNOWN";
        }
        
        gps_latitude_ = msg->latitude;
        gps_longitude_ = msg->longitude;
        gps_altitude_ = msg->altitude;
    }
    
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        last_imu_time_ = this->get_clock()->now();
        imu_status_ = "ACTIVE";
        
        // Check for reasonable IMU data
        double accel_magnitude = sqrt(
            msg->linear_acceleration.x * msg->linear_acceleration.x +
            msg->linear_acceleration.y * msg->linear_acceleration.y +
            msg->linear_acceleration.z * msg->linear_acceleration.z
        );
        
        if (accel_magnitude < 5.0 || accel_magnitude > 15.0) {
            imu_status_ = "WARNING";
        }
    }
    
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        last_cmd_vel_time_ = this->get_clock()->now();
        current_linear_vel_ = msg->linear.x;
        current_angular_vel_ = msg->angular.z;
    }
    
    void missionStatusCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        mission_status_ = msg->data;
    }
    
    void publishDiagnostics()
    {
        auto now = this->get_clock()->now();
        auto diagnostics_msg = diagnostic_msgs::msg::DiagnosticArray();
        diagnostics_msg.header.stamp = now;
        
        // GPS Status
        auto gps_diag = diagnostic_msgs::msg::DiagnosticStatus();
        gps_diag.name = "GPS";
        gps_diag.hardware_id = "UBLOX";
        
        auto gps_age = (now - last_gps_time_).seconds();
        if (gps_age > 2.0) {
            gps_diag.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
            gps_diag.message = "GPS data timeout";
        } else if (gps_status_ == "NO_FIX") {
            gps_diag.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
            gps_diag.message = "GPS no fix";
        } else {
            gps_diag.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
            gps_diag.message = "GPS " + gps_status_;
        }
        
        diagnostics_msg.status.push_back(gps_diag);
        
        // IMU Status
        auto imu_diag = diagnostic_msgs::msg::DiagnosticStatus();
        imu_diag.name = "IMU";
        imu_diag.hardware_id = "XSENS_MTi680";
        
        auto imu_age = (now - last_imu_time_).seconds();
        if (imu_age > 1.0) {
            imu_diag.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
            imu_diag.message = "IMU data timeout";
        } else {
            imu_diag.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
            imu_diag.message = "IMU " + imu_status_;
        }
        
        diagnostics_msg.status.push_back(imu_diag);
        
        // Vehicle Status
        auto vehicle_diag = diagnostic_msgs::msg::DiagnosticStatus();
        vehicle_diag.name = "Vehicle";
        vehicle_diag.hardware_id = "Bee1";
        
        if (std::abs(current_linear_vel_) > 16.0) {  // Over speed limit
            vehicle_diag.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
            vehicle_diag.message = "Speed over limit";
        } else {
            vehicle_diag.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
            vehicle_diag.message = "Vehicle nominal";
        }
        
        diagnostics_msg.status.push_back(vehicle_diag);
        
        // Publish diagnostics
        diagnostics_pub_->publish(diagnostics_msg);
        
        // Publish system status summary
        auto status_msg = std_msgs::msg::String();
        std::stringstream ss;
        ss << "System Status - ";
        ss << "GPS: " << gps_status_ << " | ";
        ss << "IMU: " << imu_status_ << " | ";
        ss << "Mission: " << mission_status_ << " | ";
        ss << "Speed: " << std::fixed << std::setprecision(1) << current_linear_vel_ << " m/s";
        
        status_msg.data = ss.str();
        system_status_pub_->publish(status_msg);
    }
    
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
    double gps_latitude_ = 0.0;
    double gps_longitude_ = 0.0;
    double gps_altitude_ = 0.0;
    double current_linear_vel_ = 0.0;
    double current_angular_vel_ = 0.0;
    
    rclcpp::Time last_gps_time_;
    rclcpp::Time last_imu_time_;
    rclcpp::Time last_cmd_vel_time_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SystemMonitor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}