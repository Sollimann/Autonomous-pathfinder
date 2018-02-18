cmake_minimum_required(VERSION 2.8.3)
project(rosproject1)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  tf
  sensor_msgs
  cv_bridge
  image_transport
  message_generation
)

# find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV REQUIRED)


##Declare new message type
#add_message_files(
#  FILES
#  Planner.msg
#)


generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package(CATKIN_DEPENDS message_runtime roscpp std_msgs geometry_msgs)


## Your package locations should be listed before other locations
include_directories(
include
  ${catkin_INCLUDE_DIRS}
  ${openCV_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/lab2.cpp
# )

## Declare a C++ executable

add_executable(navigator_node src/navigator.cpp)
target_link_libraries(navigator_node ${catkin_LIBRARIES})
