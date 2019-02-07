cmake_minimum_required(VERSION 2.8.3)
project(rosplan_interface_tiago)

set(tiago_dir "${CMAKE_CURRENT_SOURCE_DIR}/../../../../tiago_ws/devel/share")
set(pal_detection_msgs_DIR "${tiago_dir}/pal_detection_msgs/cmake/")
set(pal_interaction_msgs_DIR "${tiago_dir}/pal_interaction_msgs/cmake/")
set(tiago_pick_demo_DIR "${tiago_dir}/tiago_pick_demo/cmake/")
set(play_motion_msgs_DIR "${tiago_dir}/play_motion_msgs/cmake/")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  std_srvs
  diagnostic_msgs
  geometry_msgs
  move_base_msgs
  rosplan_dispatch_msgs
  rosplan_knowledge_msgs
  actionlib
  mongodb_store
  rosplan_interface_ext_action
  sensor_msgs
  cv_bridge
  pal_detection_msgs
  pal_interaction_msgs
  tiago_pick_demo
  play_motion_msgs
  rosplan_ilm
)

find_package(OpenCV REQUIRED)

# find_package(Boost REQUIRED COMPONENTS
#   filesystem
# )

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

add_definitions(-Wno-deprecated-declarations) # Disable deprecated declarations warning (about std::auto_ptr)

###################################
## catkin specific configuration ##
###################################

## Declare things to be passed to dependent projects
catkin_package(
  # LIBRARIES ${PROJECT_NAME} rosplan_interface_ext_action
  INCLUDE_DIRS include ${catkin_INCLUDE_DIRS}
  CATKIN_DEPENDS roscpp rospy std_msgs std_srvs actionlib rosplan_dispatch_msgs rosplan_knowledge_msgs mongodb_store diagnostic_msgs rosplan_interface_ext_action rosplan_ilm
  DEPENDS
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  include
)

## Declare cpp executables
add_executable(rpfindperson src/rosplan_interface_findperson/RPFindPerson.cpp ../rosplan_ilm/src/kb_agent/KBAgent.cpp)
add_dependencies(rpfindperson ${catkin_EXPORTED_TARGETS})

add_executable(rptalktoperson src/rosplan_interface_talktoperson/RPTalkToPerson.cpp ../rosplan_ilm/src/kb_agent/KBAgent.cpp)
add_dependencies(rptalktoperson ${catkin_EXPORTED_TARGETS})

add_executable(rpgrasp src/rosplan_interface_grasp/RPGrasp.cpp ../rosplan_ilm/src/kb_agent/KBAgent.cpp)
add_dependencies(rpgrasp ${catkin_EXPORTED_TARGETS})

add_executable(rpplaymotion src/rosplan_interface_playmotion/RPPlayMotion.cpp ../rosplan_ilm/src/kb_agent/KBAgent.cpp)
add_dependencies(rpplaymotion ${catkin_EXPORTED_TARGETS})

## Specify libraries against which to link a library or executable target
target_link_libraries(rpfindperson ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(rptalktoperson ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(rpgrasp ${catkin_LIBRARIES})
target_link_libraries(rpplaymotion ${catkin_LIBRARIES})

#############
## Install ##
#############

# install(TARGETS rpfindperson
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# install(TARGETS rptalktoperson
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )