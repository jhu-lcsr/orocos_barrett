cmake_minimum_required(VERSION 2.8.3)
project(oro_barrett_hw)

find_package(catkin REQUIRED 
  cmake_modules 
  oro_barrett_interface
  control_toolbox 
  urdf
  kdl_parser
  rtt_ros
  rtt_ros_tools
  rtt_sensor_msgs
  angles
  )

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES oro_barrett_hw
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

find_package(Eigen REQUIRED)
find_package(OROCOS-RTT REQUIRED)
find_package(Xenomai REQUIRED)
find_package(barrett REQUIRED)
find_package(orocos_kdl REQUIRED)

include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)

include_directories(include 
  ${Eigen_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${BARRETT_INCLUDE_DIRS}
  ${Xenomai_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
  ${USE_OROCOS_INCLUDE_DIRS})

add_definitions(
  ${Eigen_DEFINITIONS}
  ${BARRETT_DEFINITIONS})

link_directories(${BARRETT_LIBRARY_DIRS})

orocos_component(barrett_hw_manager
  src/barrett_hw_manager.cpp)
message(STATUS "--- ${USE_OROCOS_LIBRARIES}")
target_link_libraries(barrett_hw_manager
  ${kdl_parser_LIBRARIES}
  ${urdf_LIBRARIES}
  ${USE_OROCOS_LIBRARIES}
  ${BARRETT_LIBRARIES}
  ${orocos_kdl_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  orocos_use_package(ocl-deployment)
  orocos_use_package(ocl-taskbrowser)
  #orocos_executable(idle_test
    #test/idle_test.cpp)
endif()

orocos_generate_package()
