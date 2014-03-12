cmake_minimum_required(VERSION 2.8.3)
project(oro_barrett_hw)

find_package(catkin REQUIRED 
  cmake_modules 
  control_toolbox 
  urdf
  kdl_parser
  rtt_ros
  rtt_ros_tools
  angles
  )

if("$ENV{OROCOS_TARGET}" STREQUAL "xenomai")

  find_package(barrett)
  find_package(Xenomai)
  find_package(Eigen REQUIRED)
  find_package(OROCOS-RTT REQUIRED)
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
  endif()

  orocos_generate_package(
    INCLUDE_DIRS include
    DEPENDS barrett
    DEPENDS_TARGETS oro_barrett_interface)
else()
  message(WARNING "libbarret or xenomai could not be found, so oro_barrett_hw is not being built.")

  orocos_generate_package(
    INCLUDE_DIRS include
    DEPENDS_TARGETS oro_barrett_interface)
endif()
