#!/bin/bash
# Beemobs Bee1 System Launch Script
# Launches all necessary components for autonomous operation

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# Configuration
WORKSPACE_DIR="$HOME/bee1_cartographer_ws"
MAP_FILE="$WORKSPACE_DIR/maps/track_map.yaml"
MODE="navigation"  # Options: mapping, localization, navigation

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        -m|--mode)
            MODE="$2"
            shift 2
            ;;
        --map)
            MAP_FILE="$2"
            shift 2
            ;;
        -h|--help)
            echo "Usage: $0 [OPTIONS]"
            echo "Options:"
            echo "  -m, --mode MODE    Launch mode: mapping, localization, navigation (default: navigation)"
            echo "  --map FILE         Map file path (default: ~/bee1_cartographer_ws/maps/track_map.yaml)"
            echo "  -h, --help         Show this help message"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            exit 1
            ;;
    esac
done

echo -e "${BLUE}🚗 Beemobs Bee1 System Launcher${NC}"
echo -e "${YELLOW}Mode: $MODE${NC}"
if [[ "$MODE" != "mapping" ]]; then
    echo -e "${YELLOW}Map: $MAP_FILE${NC}"
fi
echo ""

# Check if workspace exists
if [ ! -d "$WORKSPACE_DIR" ]; then
    echo -e "${RED}❌ Workspace not found: $WORKSPACE_DIR${NC}"
    exit 1
fi

# Change to workspace directory
cd "$WORKSPACE_DIR"

# Source ROS2 and workspace
source /opt/ros/humble/setup.bash
source install/setup.bash

# Pre-flight checks
echo -e "${BLUE}🔍 Running pre-flight checks...${NC}"

# Check if nodes are built
if [ ! -f "install/bee1_cartographer/lib/bee1_cartographer/geojson_parser" ]; then
    echo -e "${RED}❌ Package not built. Run: colcon build${NC}"
    exit 1
fi

# Check hardware connections
echo -e "${YELLOW}Checking hardware connections...${NC}"

# Check LiDAR connection
if ping -c 1 -W 1 192.168.1.201 >/dev/null 2>&1; then
    echo -e "${GREEN}✅ LiDAR connected${NC}"
else
    echo -e "${YELLOW}⚠️  LiDAR not detected (will use simulated data)${NC}"
fi

# Check GPS device
if [ -e /dev/gps ]; then
    echo -e "${GREEN}✅ GPS device found${NC}"
else
    echo -e "${YELLOW}⚠️  GPS device not found${NC}"
fi

# Check IMU device
if [ -e /dev/imu ]; then
    echo -e "${GREEN}✅ IMU device found${NC}"
else
    echo -e "${YELLOW}⚠️  IMU device not found${NC}"
fi

# Check map file for navigation modes
if [[ "$MODE" != "mapping" ]] && [ ! -f "$MAP_FILE" ]; then
    echo -e "${RED}❌ Map file not found: $MAP_FILE${NC}"
    echo -e "${YELLOW}💡 Run mapping mode first to create a map${NC}"
    exit 1
fi

echo ""
echo -e "${GREEN}✅ Pre-flight checks completed${NC}"
echo ""

# Launch based on mode
case $MODE in
    "mapping")
        echo -e "${BLUE}🗺️  Starting MAPPING mode...${NC}"
        echo -e "${YELLOW}Instructions:${NC}"
        echo "1. Drive the vehicle around the track"
        echo "2. Monitor RViz for map building progress"
        echo "3. Save map when complete: ros2 run nav2_map_server map_saver_cli -f $WORKSPACE_DIR/maps/track_map"
        echo ""
        echo -e "${GREEN}Launching...${NC}"
        ros2 launch bee1_cartographer bee1_mapping.launch.py
        ;;
    
    "localization")
        echo -e "${BLUE}📍 Starting LOCALIZATION mode...${NC}"
        echo -e "${YELLOW}Using map: $MAP_FILE${NC}"
        echo ""
        echo -e "${GREEN}Launching...${NC}"
        ros2 launch bee1_cartographer bee1_localization.launch.py map:="$MAP_FILE"
        ;;
    
    "navigation")
        echo -e "${BLUE}🎯 Starting AUTONOMOUS NAVIGATION mode...${NC}"
        echo -e "${YELLOW}Using map: $MAP_FILE${NC}"
        echo -e "${YELLOW}Mission will start automatically...${NC}"
        echo ""
        echo -e "${RED}⚠️  SAFETY REMINDER:${NC}"
        echo "- Emergency stop button ready"
        echo "- Clear path to waypoints"
        echo "- Manual override available"
        echo ""
        read -p "Press ENTER to start autonomous mission..."
        echo ""
        echo -e "${GREEN}🚀 Launching autonomous navigation...${NC}"
        ros2 launch bee1_cartographer bee1_navigation.launch.py map:="$MAP_FILE"
        ;;
    
    *)
        echo -e "${RED}❌ Invalid mode: $MODE${NC}"
        echo "Valid modes: mapping, localization, navigation"
        exit 1
        ;;
esac