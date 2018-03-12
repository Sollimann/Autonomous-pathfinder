cmake_minimum_required(VERSION 2.8.3)
project(prj-grp-01)

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

find_package(Armadillo REQUIRED)
find_package(OpenCV REQUIRED)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(CATKIN_DEPENDS message_runtime roscpp std_msgs geometry_msgs sensor_msgs)

include_directories(
include
  ${catkin_INCLUDE_DIRS}
  ${openCV_INCLUDE_DIRS}
)

## Declare C++ executables:
add_executable(path_planner_node src/path_planner.cpp)
target_link_libraries(path_planner_node ${catkin_LIBRARIES})

add_executable(node_distance_logger src/node_distance_logger.cpp)
target_link_libraries(node_distance_logger ${catkin_LIBRARIES})

add_executable(node_controller src/node_controller.cpp)
target_link_libraries(node_controller ${catkin_LIBRARIES})

install(DIRECTORY launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
