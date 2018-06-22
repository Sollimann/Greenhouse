cmake_minimum_required(VERSION 2.8.3)
project(ultrasonic_device)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  serial
  thorvald_msgs
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

# Change compiler

#add_compile_options(-std=c++11)


# Add new nodes

add_executable(ultrasonic_receive_node src/ultrasonic_receive_node.cpp)
add_dependencies(ultrasonic_receive_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ultrasonic_receive_node ${catkin_LIBRARIES})
