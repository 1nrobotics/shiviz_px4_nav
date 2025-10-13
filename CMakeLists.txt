cmake_minimum_required(VERSION 3.10.2)

project(px4_offboard)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Threads REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  roscpp
  std_msgs
  tf2
  tf2_ros
  eigen_conversions
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs mavros_msgs roscpp std_msgs tf2 tf2_ros
)

# Add the px4_offboard_node subdirectory
add_subdirectory(src/px4_offboard_node)