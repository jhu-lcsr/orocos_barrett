cmake_minimum_required(VERSION 2.8.3)
project(oro_barrett_interface)

find_package(catkin REQUIRED COMPONENTS rtt_ros)

orocos_generate_package(
  INCLUDE_DIRS include
  DEPENDS_TARGETS rtt_oro_barrett_msgs
  )
