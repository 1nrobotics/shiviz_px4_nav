// Copyright 2024 1nrobotics
//
// Main entry point for Shiviz PX4 Navigation node

#include "waypoint_navigator/waypoint_navigator.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "shiviz_px4_nav");
  ros::NodeHandle nh("~");
  waypoint_navigator::WaypointNavigator waypoint_navigator(nh);
  ros::spin();
  return 0;
}