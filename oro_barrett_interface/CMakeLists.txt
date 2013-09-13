cmake_minimum_required(VERSION 2.8.3)
project(oro_barrett_interface)

find_package(catkin REQUIRED)

find_package(barrett)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES oro_barrett_interface
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

