cmake_minimum_required(VERSION 2.8.3)
project(rtt_oro_barrett_msgs)

find_package(catkin REQUIRED COMPONENTS rtt_roscomm oro_barrett_msgs)

ros_generate_rtt_typekit(oro_barrett_msgs)
ros_generate_rtt_service_proxies(oro_barrett_msgs)

orocos_generate_package(
  DEPENDS oro_barrett_msgs
  DEPENDS_TARGETS rtt_roscomm  rtt_std_msgs
)
