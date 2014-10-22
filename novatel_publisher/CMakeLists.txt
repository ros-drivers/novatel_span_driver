cmake_minimum_required(VERSION 2.8.3)
project(novatel_publisher)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  novatelcentral
  novatel_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf
)
#orocos_kdl

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

