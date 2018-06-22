cmake_minimum_required(VERSION 2.8.3)
project(mag_tape)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  thorvald_base
  ultrasonic_device
  ##thorvald_msgs
)


###########
## Build ##
###########

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES thorvald_io_serial
#  CATKIN_DEPENDS roscpp rospy serial thorvald_msgs
#  DEPENDS system_lib
)


## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})

add_executable(mag_tape_node src/mag_tape_node.cpp)
add_dependencies(mag_tape_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(mag_tape_node ${catkin_LIBRARIES})


