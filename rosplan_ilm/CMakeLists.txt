cmake_minimum_required(VERSION 2.8.3)
project(rosplan_ilm)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosplan_dispatch_msgs
  rosplan_knowledge_msgs
  rosplan_ilm_msgs
  diagnostic_msgs
  std_srvs
)


## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

###################################
## catkin specific configuration ##
###################################

## Declare things to be passed to dependent projects (must add this else executable will not be added)
catkin_package(
  INCLUDE_DIRS include ${catkin_INCLUDE_DIRS}
  CATKIN_DEPENDS roscpp rosplan_dispatch_msgs rosplan_knowledge_msgs rosplan_ilm_msgs diagnostic_msgs std_srvs
  DEPENDS
)


###########
## Build ##
###########

# Disable deprecated declarations warning (about std::auto_ptr)
# add_definitions(-Wno-deprecated-declarations)

include_directories(include 
  ${catkin_INCLUDE_DIRS})

add_executable(rexd_client src/TCP/rexd_client.cpp)
add_dependencies(rexd_client ${catkin_EXPORTED_TARGETS})

## Specify libraries against which to link a library or executable target
target_link_libraries(rexd_client
  ${catkin_LIBRARIES}
)

add_executable(rpstatemonitor src/state_monitor/RPStateMonitor.cpp src/kb_agent/KBAgent.cpp)
add_dependencies(rpstatemonitor ${catkin_EXPORTED_TARGETS})

target_link_libraries(rpstatemonitor
  ${catkin_LIBRARIES}
)