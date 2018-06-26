cmake_minimum_required(VERSION 2.8.3)
project(semi_automatic)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  thorvald_base
  ultrasonic_device
  mag_tape
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
##include_directories(include/${PROJECT_NAME})

catkin_package(INCLUDE_DIRS include)
include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})


# Add new nodes

add_executable(semi_automatic_node src/semi_automatic_node.cpp)
add_dependencies(semi_automatic_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(semi_automatic_node ${catkin_LIBRARIES})


