cmake_minimum_required(VERSION 3.0.2)
project(slip_detector)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  celex5_msgs
  cv_bridge
  roscpp
  sensor_msgs
  std_msgs
)
find_package(OpenCV REQUIRED)

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   celex5_msgs#   sensor_msgs#   std_msgs
# )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES slip_detector
#  CATKIN_DEPENDS celex5_msgs cv_bridge roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

set(CeleX ${CMAKE_SOURCE_DIR}/slip_detector/SDK)
set(CeleX_LIB CeleX)

include_directories(${CeleX}/include)
link_directories(${CeleX}/lib/Linux/x64)

#include_directories(${CeleX}/include)
#link_directories(${CeleX}/lib/Linux/x64)

# add_library(${PROJECT_NAME} src/slip_detector_ros.cpp)
# target_link_libraries(${PROJECT_NAME}
#   ${catkin_LIBRARIES}
#   ${CeleX_LIB}
# )

add_executable(${PROJECT_NAME}_node node/slip_detector_ros_node.cpp
src/slip_detector.cpp
src/event_cnt_detector.cpp
src/of_direction_detector.cpp
src/loop_detector.cpp
)

target_link_libraries(${PROJECT_NAME}_node
# ${PROJECT_NAME}
      ${catkin_LIBRARIES}
      ${OpenCV_LIBRARIES}
      ${CeleX_LIB}
)


add_executable(${PROJECT_NAME}_node_cb node/slip_detector_ros_node_cb.cpp
src/slip_detector.cpp
src/event_cnt_detector.cpp
src/of_direction_detector.cpp
src/loop_detector.cpp
)

target_link_libraries(${PROJECT_NAME}_node_cb
# ${PROJECT_NAME}
      ${catkin_LIBRARIES}
      ${OpenCV_LIBRARIES}
      ${CeleX_LIB}
)
