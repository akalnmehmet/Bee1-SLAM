cmake_minimum_required(VERSION 3.8)
project(bee1_cartographer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(diagnostic_msgs REQUIRED)

# Include directories
include_directories(include)

# GeoJSON Parser Node
add_executable(geojson_parser src/geojson_parser.cpp)
ament_target_dependencies(geojson_parser
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  builtin_interfaces
)

# Mission Executor Node
add_executable(mission_executor src/mission_executor.cpp)
ament_target_dependencies(mission_executor
  rclcpp
  rclcpp_action
  nav2_msgs
  geometry_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  builtin_interfaces
)

# Vehicle Controller Node
add_executable(vehicle_controller src/vehicle_controller.cpp)
ament_target_dependencies(vehicle_controller
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
  builtin_interfaces
)

# Hardware Interface Node
add_executable(hardware_interface src/hardware_interface.cpp)
ament_target_dependencies(hardware_interface
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  builtin_interfaces
)

# System Monitor Node
add_executable(system_monitor src/system_monitor.cpp)
ament_target_dependencies(system_monitor
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  diagnostic_msgs
  builtin_interfaces
)

# Install targets
install(TARGETS
  geojson_parser
  mission_executor
  vehicle_controller
  hardware_interface
  system_monitor
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY
  config
  launch
  urdf
  scripts
  test
  DESTINATION share/${PROJECT_NAME}
)

# Install maps directory (if exists)
install(DIRECTORY maps/
  DESTINATION share/${PROJECT_NAME}/maps
  OPTIONAL
)

# Install Python scripts as executable
install(PROGRAMS
  scripts/setup_bee1.sh
  scripts/launch_all.sh
  DESTINATION lib/${PROJECT_NAME}
)

# Install udev rules
install(FILES
  config/udev/99-bee1-devices.rules
  DESTINATION share/${PROJECT_NAME}/udev
)

# Install network config
install(FILES
  config/network/velodyne-network.yaml
  DESTINATION share/${PROJECT_NAME}/network
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  
  # Add Python tests
  ament_add_pytest_test(test_basic_functionality test/test_basic_functionality.py)
  
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()cmake_minimum_required(VERSION 3.8)
project(bee1_cartographer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

ament_package()
