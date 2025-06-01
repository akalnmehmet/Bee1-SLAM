#ifndef BEE1_CARTOGRAPHER__GEOJSON_PARSER_HPP_
#define BEE1_CARTOGRAPHER__GEOJSON_PARSER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>
#include <string>
#include <memory>
#include <cmath>

struct Waypoint {
    double latitude;
    double longitude;
    double local_x;
    double local_y;
    double heading;
    std::string name;
};

class GeoJSONParser : public rclcpp::Node
{
public:
    explicit GeoJSONParser(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    ~GeoJSONParser() = default;

private:
    void loadWaypoints();
    void publishPath();
    void convertToUTM(double lat, double lon, double& x, double& y);
    geometry_msgs::msg::PoseStamped createPoseStamped(const Waypoint& wp);
    
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
    rclcpp::TimerPtr timer_;
    
    std::vector<Waypoint> waypoints_;
    Waypoint datum_;
    
    // Datum coordinates (from GeoJSON)
    static constexpr double DATUM_LAT = 40.7903314;
    static constexpr double DATUM_LON = 29.50896659;
};

#endif  // BEE1_CARTOGRAPHER__GEOJSON_PARSER_HPP_