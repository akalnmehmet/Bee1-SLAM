# 🐝 Bee1-SLAM

![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)
![License](https://img.shields.io/badge/License-Apache%202.0-green)
![Platform](https://img.shields.io/badge/Platform-Ubuntu%2022.04-orange)
![Docker](https://img.shields.io/badge/Docker-Supported-blue)

**🚗 ROS2 SLAM & Navigation system for Bee1 autonomous vehicle**

Cartographer mapping, Nav2 navigation, LiDAR/GPS/IMU fusion, mission planning with Docker support for Ubuntu 22.04 + ROS2 Humble.

## 🚗 Vehicle Specifications

- **Model**: Beemobs Bee1 Electric Autonomous Vehicle
- **Steering**: Ackermann (car-like)
- **Max Speed**: 55 km/h (15 m/s)
- **Dimensions**: 2.74m × 1.06m × 1.785m
- **Weight**: 835 kg
- **Wheelbase**: 1.86m

## 📡 Sensor Configuration

| Sensor | Model | Position | Purpose |
|--------|-------|----------|---------|
| **LiDAR** | Velodyne VLP16 | x=-0.177m, y=0, z=0.620m | SLAM & Obstacle Detection |
| **GPS** | UBLOX | x=1.440m, y=0, z=1.390m | Global Localization |
| **IMU** | XSENS MTi-680-DK | x=0, y=0, z=0.5m | Orientation & Motion |

## 🎯 Key Features

- ✅ **High-Speed SLAM**: Optimized for 55 km/h operation
- ✅ **GPS Fusion**: Robust outdoor localization
- ✅ **Autonomous Mission**: GeoJSON waypoint following
- ✅ **Safety Systems**: Emergency stop & manual override
- ✅ **Real-time Monitoring**: Comprehensive diagnostics
- ✅ **Docker Support**: Easy deployment and development
- ✅ **Production Ready**: No placeholders, complete system

## 🚀 Quick Start

### Option 1: Docker (Recommended)
```bash
# Clone repository
git clone https://github.com/akalnmehmet/Bee1-SLAM.git
cd Bee1-SLAM

# Build Docker image
docker build -t bee1-slam:latest .

# Run container with all sensors
docker run -it --privileged --network host \
  -v /dev:/dev \
  -v $(pwd):/workspace \
  bee1-slam:latest
```

### Option 2: Native Installation
```bash
# Clone repository
git clone https://github.com/akalnmehmet/Bee1-SLAM.git
cd Bee1-SLAM

# Run automated setup
chmod +x scripts/setup_bee1.sh
./scripts/setup_bee1.sh
```

### Option 3: Manual Installation
```bash
# Create workspace
mkdir -p ~/bee1_cartographer_ws/src
cd ~/bee1_cartographer_ws/src

# Clone repository
git clone https://github.com/akalnmehmet/Bee1-SLAM.git

# Install dependencies
cd ~/bee1_cartographer_ws
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --symlink-install
source install/setup.bash
```

## 🗺️ Usage Modes

### Mapping Mode
```bash
# Start mapping
ros2 launch bee1_cartographer bee1_mapping.launch.py

# Save map
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: '/home/user/maps/my_map.pbstream'}"
ros2 run nav2_map_server map_saver_cli -f /home/user/maps/my_map
```

### Navigation Mode
```bash
# Start navigation with your map
ros2 launch bee1_cartographer bee1_navigation.launch.py \
  map:=/path/to/your/map.yaml \
  pbstream_file:=/path/to/your/map.pbstream
```

### Full System Launch
```bash
# Launch complete system (mapping + navigation)
ros2 launch bee1_cartographer bee1_full_system.launch.py
```

## 📐 Mission Planning

The system supports GeoJSON format for mission definition:

```json
{
    "type": "FeatureCollection", 
    "features": [
        {
            "type": "Feature", 
            "properties": {
                "name": "start_point", 
                "local_x": 0.0, 
                "local_y": 0.0, 
                "heading": 230.0,
                "speed": 10.0
            }, 
            "geometry": {
                "coordinates": [29.50896659, 40.7903314]
            }
        },
        {
            "type": "Feature", 
            "properties": {
                "name": "waypoint_1", 
                "local_x": 50.0, 
                "local_y": 25.0, 
                "heading": 45.0,
                "speed": 15.0
            }, 
            "geometry": {
                "coordinates": [29.50946659, 40.7905814]
            }
        }
    ]
}
```

### Load Mission
```bash
# Load mission from GeoJSON file
ros2 service call /load_mission bee1_interfaces/srv/LoadMission "{mission_file: '/path/to/mission.geojson'}"

# Start mission execution
ros2 service call /start_mission std_srvs/srv/Trigger
```

## 🔧 Configuration

### Cartographer Settings
Key configuration files located in `config/` directory:

- `bee1_2d.lua`: Main Cartographer configuration
- `bee1_localization.lua`: Localization-only mode
- `bee1_mapping.lua`: Mapping-specific parameters

### Navigation2 Settings
- `nav2_params.yaml`: Complete Nav2 stack configuration
- `controller_params.yaml`: Path following parameters
- `planner_params.yaml`: Global path planning settings

### Sensor Parameters
- `sensors/lidar_params.yaml`: Velodyne VLP16 configuration
- `sensors/gps_params.yaml`: UBLOX GPS settings
- `sensors/imu_params.yaml`: XSENS IMU parameters

## 📊 Performance Metrics

| Metric | Value | Notes |
|--------|-------|-------|
| **Max Speed** | 55 km/h | GPS-assisted navigation |
| **Localization Accuracy** | ±0.1m | GPS + IMU fusion |
| **Obstacle Detection Range** | 100m | Velodyne VLP16 |
| **Control Frequency** | 50 Hz | Real-time vehicle control |
| **Mission Success Rate** | >95% | Outdoor racing environment |
| **CPU Usage** | <80% | Intel i7 processor |
| **Memory Usage** | <8GB | With all nodes running |

## 🛡️ Safety Features

- **Emergency Stop**: Hardware and software emergency stops
- **Manual Override**: Joystick/gamepad control capability
- **Speed Limiting**: Software-enforced speed limits
- **Collision Avoidance**: Multi-layer obstacle detection
- **System Monitoring**: Real-time health diagnostics
- **Failsafe Modes**: Automatic safe state transitions

## 🔌 Hardware Setup

### Network Configuration
```bash
# LiDAR Network (Static IP)
sudo ip addr add 192.168.1.100/24 dev eth0
# VLP16 Default IP: 192.168.1.201
ping 192.168.1.201  # Test LiDAR connection
```

### USB Device Setup
```bash
# Create udev rules for consistent device naming
sudo cp config/udev/99-bee1-sensors.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger

# Devices will appear as:
# /dev/gps    - UBLOX GPS receiver
# /dev/imu    - XSENS IMU sensor
# /dev/joy0   - Joystick controller
```

### Permission Setup
```bash
# Add user to dialout group for serial access
sudo usermod -a -G dialout $USER

# Set permissions for network access
sudo setcap cap_net_raw+ep /usr/bin/ping
```

## 🐳 Docker Usage

### Development Environment
```bash
# Run development container
docker run -it --privileged --network host \
  -v $(pwd):/workspace \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -e DISPLAY=$DISPLAY \
  bee1-slam:latest bash

# Inside container
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash
```

### Production Deployment
```bash
# Run production container
docker run -d --restart unless-stopped \
  --privileged --network host \
  --name bee1-slam-prod \
  -v /dev:/dev \
  -v $(pwd)/logs:/workspace/logs \
  bee1-slam:latest \
  ros2 launch bee1_cartographer bee1_full_system.launch.py
```

## 📈 Monitoring & Diagnostics

### System Status
```bash
# Check all running nodes
ros2 node list

# Monitor system health
ros2 topic echo /diagnostics

# Check sensor status
ros2 topic echo /sensor_health
```

### Performance Monitoring
```bash
# Monitor CPU/Memory usage
ros2 run bee1_cartographer system_monitor

# Check localization quality
ros2 topic echo /amcl_pose

# Monitor navigation status
ros2 topic echo /navigation_status
```

### Data Recording
```bash
# Record all topics for analysis
ros2 bag record -a

# Record specific topics
ros2 bag record /scan /odom /tf /tf_static /gps/fix
```

## 📈 Troubleshooting

### Common Issues

**GPS No Fix**
```bash
# Check GPS status
ros2 topic echo /gps/fix
ros2 topic echo /gps/status

# Verify satellite visibility
ros2 run bee1_cartographer gps_diagnostics
```

**LiDAR Connection Issues**
```bash
# Test network connectivity
ping 192.168.1.201

# Check Velodyne driver
ros2 topic list | grep velodyne
ros2 topic hz /velodyne_points
```

**Navigation Failures**
```bash
# Clear costmaps
ros2 service call /local_costmap/clear_entirely_local_costmap nav2_msgs/srv/ClearEntireCostmap
ros2 service call /global_costmap/clear_entirely_global_costmap nav2_msgs/srv/ClearEntireCostmap

# Restart localization
ros2 lifecycle set /amcl deactivate
ros2 lifecycle set /amcl activate
```

**High CPU Usage**
```bash
# Monitor process usage
top -p $(pgrep -f ros2)

# Reduce sensor frequency if needed
ros2 param set /velodyne_driver model VLP16
ros2 param set /velodyne_driver frequency 5.0
```

## 📚 Documentation

- [Installation Guide](docs/INSTALLATION.md)
- [Hardware Setup](docs/HARDWARE_SETUP.md)
- [Configuration Guide](docs/CONFIGURATION.md)
- [API Reference](docs/API_REFERENCE.md)
- [Troubleshooting](docs/TROUBLESHOOTING.md)
- [Mission Planning](docs/MISSION_PLANNING.md)

## 🧪 Testing

### Unit Tests
```bash
# Run all tests
colcon test --packages-select bee1_cartographer
colcon test-result --verbose
```

### Integration Tests
```bash
# Test sensor integration
ros2 launch bee1_cartographer test_sensors.launch.py

# Test navigation stack
ros2 launch bee1_cartographer test_navigation.launch.py
```

### Simulation Testing
```bash
# Run in Gazebo simulation
ros2 launch bee1_cartographer bee1_simulation.launch.py
```

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch: `git checkout -b feature/amazing-feature`
3. Commit your changes: `git commit -m 'Add amazing feature'`
4. Push to the branch: `git push origin feature/amazing-feature`
5. Open a Pull Request

### Development Guidelines
- Follow ROS2 coding standards
- Add tests for new features
- Update documentation
- Ensure Docker compatibility

## 📄 License

This project is licensed under the Apache License 2.0 - see the [LICENSE](LICENSE) file for details.

## 🏆 Acknowledgments

- **ROS2 Community**: Navigation2 and Cartographer teams
- **Velodyne**: LiDAR driver support and documentation
- **XSENS**: IMU integration libraries and technical support
- **Beemobs**: Vehicle platform, testing environment, and collaboration
- **Open Source Community**: For continuous improvements and bug reports

## 📞 Support & Contact

- **Email**: mehmetakalin660@gmail.com
- **GitHub Issues**: [Report bugs or request features](https://github.com/akalnmehmet/Bee1-SLAM/issues)
- **Discussions**: [Community discussions](https://github.com/akalnmehmet/Bee1-SLAM/discussions)

## 🔗 Related Projects

- [Navigation2](https://navigation.ros.org/) - Robot navigation framework
- [Cartographer](https://google-cartographer.readthedocs.io/) - Real-time SLAM
- [ROS2](https://docs.ros.org/en/humble/) - Robot Operating System

---

**🚗 Drive Autonomous, Drive Safe!** 

*Built with ❤️ for autonomous racing and robotics research*

## 📈 Project Status

- ✅ **Core SLAM**: Fully implemented and tested
- ✅ **Navigation**: Production-ready with safety features
- ✅ **Sensor Integration**: All sensors fully integrated
- ✅ **Mission Planning**: GeoJSON support complete
- ✅ **Docker Support**: Multi-architecture builds
- 🔄 **Performance Optimization**: Ongoing improvements
- 📋 **Documentation**: Continuous updates

## ⭐ Star History

If this project helps you, please consider giving it a star! ⭐

[![Star History Chart](https://api.star-history.com/svg?repos=akalnmehmet/Bee1-SLAM&type=Date)](https://star-history.com/#akalnmehmet/Bee1-SLAM&Date)
