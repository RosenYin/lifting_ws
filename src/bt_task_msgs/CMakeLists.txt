cmake_minimum_required(VERSION 2.8.3)
project(bt_task_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  message_runtime
  std_msgs
  actionlib_msgs
  actionlib
)

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   LiftMotorMsg.msg
 )

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   LiftInterfaceSrv.srv
   LiftMotorSrv.srv
 )

# add_action_files(
#   DIRECTORY action
#   FILES     Fibonacci.action
# )
## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   actionlib_msgs
 )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES xarm_msgs
  CATKIN_DEPENDS std_msgs message_generation message_runtime 
#  DEPENDS system_lib
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)
