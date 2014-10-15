cmake_minimum_required(VERSION 2.8.3)
project(oro_barrett_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs message_generation actionlib actionlib_msgs)

add_action_files(
  DIRECTORY action
  FILES
  SetHome.action
  SetMode.action
  BHandGrasp.action
  BHandRelease.action)

add_message_files(
  FILES
  BHandCmd.msg
  BHandCmdMode.msg
  BHandStatus.msg
  SafetyMode.msg
  RunMode.msg
  BarrettStatus.msg)

generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs)

catkin_package(
  CATKIN_DEPENDS message_runtime
  )


