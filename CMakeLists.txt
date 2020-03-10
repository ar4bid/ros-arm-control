cmake_minimum_required(VERSION 2.8.3)
project(arm_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED 
  COMPONENTS
  roscpp
  rospy
  std_msgs
  sabertooth
  cui_encoders
  qset_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################


## Generate messages in the 'msg' folder

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
#generate_messages(
#  DEPENDENCIES
#  std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################


## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

catkin_package(
  INCLUDE_DIRS 
    src
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS 
    roscpp 
    std_msgs 
#    message_runtime
#  DEPENDS system_lib
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(joint_control_node src/joint_control_node.cpp)
add_executable(position_control_node src/position_control_node.cpp)
add_executable(angle_control_node src/angle_control_node.cpp)
##Uncomment below line when limit switches are added--------------------------------------------------------
## add_executable(limit_switch_node src/limitSwitch.cpp)

## Declare a C++ library
add_library(position_control src/position_control.cpp)
add_library(angle_control src/angle_control.cpp)
add_library(joint_control src/joint_control.cpp)
add_library(Arm src/Arm.cpp)
 
set_target_properties(joint_control_node PROPERTIES OUTPUT_NAME joint_control_node  PREFIX "")
set_target_properties(position_control_node PROPERTIES OUTPUT_NAME position_control_node PREFIX "")
set_target_properties(angle_control_node PROPERTIES OUTPUT_NAME angle_control_node PREFIX "")
##Uncomment below line when limit switches are added--------------------------------------------------------
## set_target_properties(limit_switch_node PROPERTIES OUTPUT_NAME limit_switch PREFIX "")

## same as for the library above
add_dependencies(joint_control_node  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(position_control_node  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(angle_control_node  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

##Uncomment below line when limit switches are added--------------------------------------------------------
## add_dependencies(limit_switch_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against

target_link_libraries(joint_control_node joint_control Arm ${catkin_LIBRARIES})
target_link_libraries(position_control_node position_control angle_control Arm ${catkin_LIBRARIES})
target_link_libraries(angle_control_node angle_control Arm ${catkin_LIBRARIES})


##Uncomment below line when limit switches are added--------------------------------------------------------
##target_link_libraries(limit_switch_node ${catkin_LIBRARIES})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(${PROJECT_NAME}-test test/utest.cpp)
  target_link_libraries(${PROJECT_NAME}-test joint_control angle_control position_control Arm ${catkin_LIBRARIES})
 endif()

add_dependencies(${PROJECT_NAME}-test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
