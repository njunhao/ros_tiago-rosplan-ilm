cmake_minimum_required(VERSION 2.8.3)
project(rosplan_interface_ext_action)

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
  rosplan_ilm
)

find_package(Boost REQUIRED COMPONENTS
  filesystem
)


## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

add_definitions(-Wno-deprecated-declarations) # Disable deprecated declarations warning (about std::auto_ptr)

###################################
## catkin specific configuration ##
###################################

## Declare things to be passed to dependent projects
catkin_package(
  LIBRARIES ${PROJECT_NAME}
  INCLUDE_DIRS include ${catkin_INCLUDE_DIRS}
  CATKIN_DEPENDS roscpp rospy std_msgs std_srvs actionlib rosplan_dispatch_msgs rosplan_knowledge_msgs mongodb_store diagnostic_msgs
  DEPENDS
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

## Declare cpp executables
add_executable(simulatedProbAction src/ActionInterface/RPProbSimulatedActionInterface.cpp src/ActionInterface/RPProbActionInterface.cpp ../rosplan_ilm/src/kb_agent/KBAgent.cpp)
add_dependencies(simulatedProbAction ${catkin_EXPORTED_TARGETS})

add_executable(rplocalise src/rosplan_interface_localise/RPLocalise.cpp src/ActionInterface/RPProbActionInterface.cpp ../rosplan_ilm/src/kb_agent/KBAgent.cpp)
add_dependencies(rplocalise ${catkin_EXPORTED_TARGETS})

add_executable(rpexplore src/rosplan_interface_explore/RPExplore.cpp src/ActionInterface/RPProbActionInterface.cpp ../rosplan_ilm/src/kb_agent/KBAgent.cpp)
add_dependencies(rpexplore ${catkin_EXPORTED_TARGETS})

add_executable(rpmove src/rosplan_interface_move/RPMove.cpp src/ActionInterface/RPProbActionInterface.cpp ../rosplan_ilm/src/kb_agent/KBAgent.cpp)
add_dependencies(rpmove ${catkin_EXPORTED_TARGETS})


## Specify libraries against which to link a library or executable target
target_link_libraries(simulatedProbAction ${catkin_LIBRARIES})
target_link_libraries(rplocalise ${catkin_LIBRARIES})
target_link_libraries(rpexplore ${catkin_LIBRARIES})
target_link_libraries(rpmove ${catkin_LIBRARIES} ${Boost_LIBRARIES})


# not rquired to do this because catkin_package is doing it
add_library(rosplan_interface_ext_action 
  src/ActionInterface/RPProbActionInterface.cpp 
  src/rosplan_interface_explore/RPExplore.cpp 
  src/rosplan_interface_search/RPSearch.cpp
  src/rosplan_interface_talk/RPTalk.cpp
)
add_dependencies(rosplan_interface_ext_action ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############

# install(TARGETS rplocalise
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# install(TARGETS rpexplore
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )