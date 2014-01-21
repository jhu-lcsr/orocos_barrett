cmake_minimum_required(VERSION 2.8.3)
project(oro_barrett_msgs)

find_package(catkin REQUIRED COMPONENTS 
  rtt_roscomm message_generation)

add_message_files(
  FILES
  BHandCmd.msg)
generate_messages()

catkin_package()

# Generate typekits for ros .msg files
ros_generate_rtt_typekit(oro_barrett_msgs)
# Generate the plugin which makes the services in my_msgs available
ros_generate_rtt_service_proxies(oro_barrett_msgs)

orocos_generate_package(
  DEPENDS_TARGETS rtt_roscomm)

