cmake_minimum_required(VERSION 2.8.3)
project(aws_robomaker_small_house_world)

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
  house_light_msgs.msg
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

add_library(TurnOnLightPlugin SHARED plugin/light_plugin.cc)
target_link_libraries(TurnOnLightPlugin
  ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES}
)

add_library(LightPlugin SHARED plugin/house_light.cc)
target_link_libraries(LightPlugin
  ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES}
)
