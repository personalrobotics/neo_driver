cmake_minimum_required(VERSION 2.8.3)
project(neo_watchdogs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  neo_relayboard
  roscpp
  neo_msgs
  move_base_msgs
)



################################################
## Declare ROS messages, services and actions ##
################################################




###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES neo_watchdogs
  CATKIN_DEPENDS 
  neo_relayboard 
  roscpp
  move_base_msgs
  neo_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(watch_power src/watch_power.cpp)
add_executable(watch_scanner_stop src/watch_scanner_stop.cpp)
add_executable(watch_temp src/watch_temp.cpp)

target_link_libraries(watch_power
   ${catkin_LIBRARIES})
target_link_libraries(watch_scanner_stop
   ${catkin_LIBRARIES})
target_link_libraries(watch_temp
   ${catkin_LIBRARIES})



#############
## Install ##
#############


#############
## Testing ##
#############


