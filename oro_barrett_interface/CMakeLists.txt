cmake_minimum_required(VERSION 2.8.3)
project(oro_barrett_interface)

find_package(catkin REQUIRED COMPONENTS
  rtt_ros
  rtt_oro_barrett_msgs)

find_package(barrett)
find_package(orocos_kdl)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS rtt_oro_barrett_msgs
)

orocos_generate_package(
  DEPENDS rtt_oro_barrett_msgs
  )
