cmake_minimum_required(VERSION 2.8.3)
project(ros_comm)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  message_generation
  std_msgs
  geometry_msgs
  sensor_msgs
  visualization_msgs
  )

add_message_files(
  FILES
  Pose2DArray.msg
)

#catkin_python_setup()

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime   
)

#install(PROGRAMS detect
#        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
