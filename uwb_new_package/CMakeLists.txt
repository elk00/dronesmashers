cmake_minimum_required(VERSION 3.5)
project(uwb_new_package)

# Clear any leftover Catkin variables (safe to include but not necessary in ROS 2)
unset(CATKIN_INSTALL_INTO_PREFIX_ROOT CACHE)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

# Add executable
add_executable(
  uwb_node
  src/parse.cpp
  src/nlink_utils.c
  src/nlink_linktrack_nodeframe0.c
  src/nlink_linktrack_nodeframe1.c
  src/nlink_linktrack_nodeframe2.c
  src/nlink_linktrack_nodeframe3.c
  src/UDPClient.cpp
  src/UDPServer.cpp
)

# Specify include directories
target_include_directories(uwb_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link dependencies
ament_target_dependencies(uwb_node rclcpp geometry_msgs)

# Install the executable
install(TARGETS uwb_node
  DESTINATION lib/${PROJECT_NAME})

# Install headers (if applicable)
install(DIRECTORY include/
  DESTINATION include)

ament_package()

