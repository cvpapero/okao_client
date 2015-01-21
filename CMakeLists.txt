cmake_minimum_required(VERSION 2.8.3)
project(okao_client)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  humans_msgs
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  visualization_msgs
  camera_info_manager
)
find_package(OpenCV REQUIRED)
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
add_service_files(
  FILES
  OkaoStack.srv
  OkaoPosSrv.srv
  LostTracking.srv
  testmsg.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  sensor_msgs
  humans_msgs
  std_msgs#   visualization_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES okao_client
  CATKIN_DEPENDS cv_bridge geometry_msgs image_transport roscpp rospy sensor_msgs std_msgs tf visualization_msgs camera_info_manager

#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
find_library(ZMQ_LIBRARIE
	NAMES libzmq.so
	HITNS /usr/local/lib/
	PATH_SUFFIXES lib
)
## Declare a cpp library
# add_library(okao_client
#   src/${PROJECT_NAME}/okao_client.cpp
# )

## Declare a cpp executable
# add_executable(okao_client_node src/okao_client_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(okao_client_node okao_client_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(okao_client_node
#   ${catkin_LIBRARIES}
# )

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
# install(TARGETS okao_client okao_client_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_okao_client.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

add_executable(OkaoClient src/OkaoClient.cpp)
target_link_libraries(OkaoClient ${catkin_LIBRARIES} ${OpenCV_LIBS} ${ZMQ_LIBRARIE} msgpack)

add_executable(okao_stack src/okao_stack.cpp)
target_link_libraries(okao_stack ${catkin_LIBRARIES})

add_executable(people_recog_info src/people_recog_info.cpp)
target_link_libraries(people_recog_info ${catkin_LIBRARIES})

add_executable(people_recog_info2 src/people_recog_info2.cpp)
target_link_libraries(people_recog_info2 ${catkin_LIBRARIES})

#add_executable(people_position_client src/people_position_client.cpp)
#target_link_libraries(people_position_client ${catkin_LIBRARIES})

#add_executable(recog_info_vision src/recog_info_vision.cpp)
#target_link_libraries(recog_info_vision ${catkin_LIBRARIES} ${OpenCV_LIBS})

#add_executable(depth_to_laser2 src/depth_to_laser2.cpp)
#target_link_libraries(depth_to_laser2 ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(face_mapping_publisher src/face_mapping_publisher.cpp)
target_link_libraries(face_mapping_publisher ${catkin_LIBRARIES})

add_executable(face_mapping_publisher2 src/face_mapping_publisher2.cpp)
target_link_libraries(face_mapping_publisher2 ${catkin_LIBRARIES})

add_executable(tracking_points src/tracking_points.cpp)
target_link_libraries(tracking_points ${catkin_LIBRARIES})

#add_executable(position_sub src/position_sub.cpp)
#target_link_libraries(position_sub ${catkin_LIBRARIES})

#add_executable(humans_pass2 src/humans_pass2.cpp)
#target_link_libraries(humans_pass2 ${catkin_LIBRARIES})

#add_executable(face_detector src/face_detector.cpp)
#target_link_libraries(face_detector ${catkin_LIBRARIES} ${OpenCV_LIBS})
