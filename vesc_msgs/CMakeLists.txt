cmake_minimum_required(VERSION 2.8.3)
project(vesc_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
)

add_message_files(
  DIRECTORY msg
  FILES
  VescState.msg
  VescStateStamped.msg
  VescCtrl.msg
  VescCtrlStamped.msg
  State.msg
  ExtendedState.msg
  Circle.msg
  Input.msg
  Reference.msg
  DataPointStamped.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS std_msgs message_runtime
)
