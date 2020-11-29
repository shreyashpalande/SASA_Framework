cmake_minimum_required(VERSION 2.8.3)
project(aws_robomaker_bookstore_world)


find_package(gazebo REQUIRED)
find_package(roscpp REQUIRED)

find_package(roscpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(catkin REQUIRED COMPONENTS
rospy
roscpp
gazebo_ros
)

add_message_files(
  FILES
  bookstore_light_msgs.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  DEPENDS
    roscpp
    gazebo_ros
    message_runtime
    std_msgs
)

include_directories(${GAZEBO_INCLUDE_DIRS})
include_directories(${roscpp_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(BookStoreLightPlugin SHARED plugin/bookstore_light.cc)
target_link_libraries(BookStoreLightPlugin
  ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES}
)

add_library(RobotLightPlugin SHARED plugin/robot_light.cc)
target_link_libraries(RobotLightPlugin
  ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES}
)
