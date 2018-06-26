cmake_minimum_required(VERSION 2.8.3)
project(semi_auto)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  std_srvs
  thorvald_base
  ultrasonic_device
  mag_tape
)

# Service files
add_service_files(FILES
 PlantMonitoring.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES 
  std_msgs
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
##include_directories(include/${PROJECT_NAME})

catkin_package(CATKIN_DEPENDS std_srvs
INCLUDE_DIRS include)
include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})


# Add new nodes

# Service
add_executable(semi_automatic_node src/semi_automatic_node.cpp)
add_dependencies(semi_automatic_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(semi_automatic_node ${catkin_LIBRARIES})

# Client
add_executable(client src/client.cpp)
add_dependencies(client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(client ${catkin_LIBRARIES})


