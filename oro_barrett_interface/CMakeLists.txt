cmake_minimum_required(VERSION 2.8.3)
project(oro_barrett_interface)

find_package(catkin REQUIRED COMPONENTS
  rtt_ros rtt_ros_tools kdl_parser cmake_modules)

find_package(Eigen REQUIRED)
find_package(orocos_kdl REQUIRED)

include_directories(
  include
  ${Eigen_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS})

add_definitions(
  ${Eigen_DEFINITIONS})

orocos_library(oro_barrett_interface
  src/utils.cpp
  src/hand_device.cpp)
target_link_libraries(oro_barrett_interface
  ${catkin_LIBRARIES}
  ${orocos_kdl_LIBRARIES})


orocos_generate_package(
  INCLUDE_DIRS include
  DEPENDS_TARGETS rtt_oro_barrett_msgs rtt_actionlib
  )

catkin_python_setup()

