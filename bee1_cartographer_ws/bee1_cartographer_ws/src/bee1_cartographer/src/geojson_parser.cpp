#include "bee1_cartographer/geojson_parser.hpp"
#include <cmath>
#include <fstream>
#include <sstream>
#include <regex>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

GeoJSONParser::GeoJSONParser(const rclcpp::NodeOptions & options)
: Node("geojson_parser", options)
{
    path_publisher_ = this->create_publisher<nav_msgs::msg::Path>(
        "mission_waypoints", 10);
    
    timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&GeoJSONParser::publishPath, this));
    
    // Initialize datum
    datum_.latitude = DATUM_LAT;
    datum_.longitude = DATUM_LON;
    datum_.local_x = 0.0;
    datum_.local_y = 0.0;
    datum_.heading = 230.0;
    datum_.name = "datum";
    
    loadWaypoints();
    
    RCLCPP_INFO(this->get_logger(), "GeoJSON Parser initialized with %zu waypoints", 
                waypoints_.size());
}

void GeoJSONParser::loadWaypoints()
{
    // Load hardcoded GeoJSON waypoints
    waypoints_.clear();
    
    // Datum/Start point
    Waypoint start;
    start.latitude = 40.7903314;
    start.longitude = 29.50896659;
    start.local_x = 0.0;
    start.local_y = 0.0;
    start.heading = 230.0;
    start.name = "start";
    waypoints_.push_back(start);
    
    // Mission waypoint 1
    Waypoint mission1;
    mission1.latitude = 40.7898375;
    mission1.longitude = 29.5085274;
    mission1.local_x = -35.48;
    mission1.local_y = -55.89;
    mission1.heading = 140.0;
    mission1.name = "gorev_1";
    waypoints_.push_back(mission1);
    
    RCLCPP_INFO(this->get_logger(), "Loaded %zu waypoints from GeoJSON", waypoints_.size());
}

void GeoJSONParser::convertToUTM(double lat, double lon, double& x, double& y)
{
    // Simple local coordinate conversion relative to datum
    // For more accurate conversion, use proj4 or similar library
    double lat_rad = lat * M_PI / 180.0;
    double lon_rad = lon * M_PI / 180.0;
    double datum_lat_rad = datum_.latitude * M_PI / 180.0;
    double datum_lon_rad = datum_.longitude * M_PI / 180.0;
    
    const double EARTH_RADIUS = 6378137.0; // WGS84 equatorial radius
    
    x = (lon_rad - datum_lon_rad) * EARTH_RADIUS * cos(datum_lat_rad);
    y = (lat_rad - datum_lat_rad) * EARTH_RADIUS;
}

geometry_msgs::msg::PoseStamped GeoJSONParser::createPoseStamped(const Waypoint& wp)
{
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = this->get_clock()->now();
    
    // Use local coordinates if available, otherwise convert from GPS
    if (wp.local_x != 0.0 || wp.local_y != 0.0) {
        pose.pose.position.x = wp.local_x;
        pose.pose.position.y = wp.local_y;
    } else {
        double x, y;
        convertToUTM(wp.latitude, wp.longitude, x, y);
        pose.pose.position.x = x;
        pose.pose.position.y = y;
    }
    pose.pose.position.z = 0.0;
    
    // Convert heading to quaternion
    tf2::Quaternion quat;
    quat.setRPY(0, 0, wp.heading * M_PI / 180.0);
    pose.pose.orientation = tf2::toMsg(quat);
    
    return pose;
}

void GeoJSONParser::publishPath()
{
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    path.header.stamp = this->get_clock()->now();
    
    for (const auto& waypoint : waypoints_) {
        path.poses.push_back(createPoseStamped(waypoint));
    }
    
    path_publisher_->publish(path);
}

// Main function for the node
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GeoJSONParser>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}