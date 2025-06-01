#!/bin/bash
# Beemobs Bee1 System Setup Script
# Run with: chmod +x setup_bee1.sh && ./setup_bee1.sh

set -e

echo "🚗 Beemobs Bee1 System Setup Started..."

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Check if running as root
if [[ $EUID -eq 0 ]]; then
   echo -e "${RED}This script should not be run as root${NC}"
   exit 1
fi

echo -e "${BLUE}1. Installing ROS2 Humble and dependencies...${NC}"

# Update system
sudo apt update && sudo apt upgrade -y

# Install ROS2 Humble if not already installed
if ! command -v ros2 &> /dev/null; then
    echo "Installing ROS2 Humble..."
    
    # Add ROS2 repository
    sudo apt install software-properties-common curl gnupg lsb-release -y
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    
    # Install ROS2
    sudo apt update
    sudo apt install ros-humble-desktop -y
else
    echo "ROS2 already installed"
fi

echo -e "${BLUE}2. Installing Bee1 dependencies...${NC}"

# Install required packages
sudo apt install -y \
    ros-humble-cartographer \
    ros-humble-cartographer-ros \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-robot-localization \
    ros-humble-velodyne \
    ros-humble-velodyne-driver \
    ros-humble-velodyne-pointcloud \
    ros-humble-pointcloud-to-laserscan \
    ros-humble-ublox \
    ros-humble-xacro \
    ros-humble-joint-state-publisher \
    ros-humble-robot-state-publisher \
    ros-humble-tf2-tools \
    ros-humble-tf2-geometry-msgs \
    python3-geojson \
    python3-pyproj \
    python3-colcon-common-extensions \
    python3-rosdep

# Initialize rosdep if not already done
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
fi
rosdep update

echo -e "${BLUE}3. Setting up workspace...${NC}"

# Create workspace
mkdir -p ~/bee1_cartographer_ws/src
cd ~/bee1_cartographer_ws

# Create maps directory
mkdir -p maps

echo -e "${BLUE}4. Setting up environment...${NC}"

# Add ROS2 setup to bashrc if not already there
if ! grep -q "source /opt/ros/humble/setup.bash" ~/.bashrc; then
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
fi

if ! grep -q "source ~/bee1_cartographer_ws/install/setup.bash" ~/.bashrc; then
    echo "source ~/bee1_cartographer_ws/install/setup.bash" >> ~/.bashrc
fi

# Add environment variables
if ! grep -q "export ROS_DOMAIN_ID=42" ~/.bashrc; then
    echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc
fi

if ! grep -q "export RMW_IMPLEMENTATION=rmw_cyclonedx_cpp" ~/.bashrc; then
    echo "export RMW_IMPLEMENTATION=rmw_cyclonedx_cpp" >> ~/.bashrc
fi

echo -e "${BLUE}5. Setting up udev rules for hardware...${NC}"

# Create udev rules for Velodyne LiDAR
sudo tee /etc/udev/rules.d/99-velodyne.rules > /dev/null <<EOF
# Velodyne LiDAR udev rules
SUBSYSTEM=="net", ATTRS{address}=="60:76:88:*", NAME="velodyne"
EOF

# Create udev rules for GPS
sudo tee /etc/udev/rules.d/99-ublox-gps.rules > /dev/null <<EOF
# UBLOX GPS udev rules
SUBSYSTEM=="tty", ATTRS{idVendor}=="1546", ATTRS{idProduct}=="01a*", SYMLINK+="gps"
EOF

# Create udev rules for IMU
sudo tee /etc/udev/rules.d/99-xsens-imu.rules > /dev/null <<EOF
# XSENS IMU udev rules
SUBSYSTEM=="tty", ATTRS{idVendor}=="2639", ATTRS{idProduct}=="*", SYMLINK+="imu"
EOF

# Reload udev rules
sudo udevadm control --reload-rules
sudo udevadm trigger

echo -e "${BLUE}6. Configuring network for LiDAR...${NC}"

# Configure static IP for Velodyne LiDAR communication
sudo tee /etc/netplan/99-velodyne.yaml > /dev/null <<EOF
network:
  version: 2
  ethernets:
    velodyne:
      addresses:
        - 192.168.1.100/24
      dhcp4: false
      optional: true
EOF

echo -e "${BLUE}7. Setting up system service...${NC}"

# Copy service file (if exists)
if [ -f "scripts/bee1_system.service" ]; then
    sudo cp scripts/bee1_system.service /etc/systemd/system/
    sudo systemctl daemon-reload
    echo "System service installed. Enable with: sudo systemctl enable bee1_system.service"
fi

echo -e "${BLUE}8. Setting up performance optimizations...${NC}"

# Set CPU governor to performance
echo 'GOVERNOR="performance"' | sudo tee /etc/default/cpufrequtils > /dev/null

# Increase network buffer sizes
sudo tee -a /etc/sysctl.conf > /dev/null <<EOF

# Bee1 Network Optimizations
net.core.rmem_max = 134217728
net.core.rmem_default = 134217728
net.core.wmem_max = 134217728
net.core.wmem_default = 134217728
net.core.netdev_max_backlog = 5000
EOF

echo -e "${BLUE}9. Building workspace...${NC}"

# Source ROS2
source /opt/ros/humble/setup.bash

# Install rosdep dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build workspace
colcon build --symlink-install

# Source workspace
source install/setup.bash

echo -e "${GREEN}✅ Beemobs Bee1 system setup completed!${NC}"
echo ""
echo -e "${YELLOW}Next steps:${NC}"
echo "1. Copy all Bee1 package files to ~/bee1_cartographer_ws/src/bee1_cartographer/"
echo "2. Run: cd ~/bee1_cartographer_ws && colcon build --symlink-install"
echo "3. Test system: ros2 launch bee1_cartographer bee1_bringup.launch.py"
echo "4. Enable auto-start: sudo systemctl enable bee1_system.service"
echo ""
echo -e "${YELLOW}Hardware setup:${NC}"
echo "- Connect Velodyne LiDAR to ethernet port"
echo "- Connect UBLOX GPS to USB port"
echo "- Connect XSENS IMU to USB port"
echo "- Reboot system to apply all changes"
echo ""
echo -e "${GREEN}🚗 Happy autonomous driving!${NC}"