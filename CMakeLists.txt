cmake_minimum_required(VERSION 2.8.3)
project(basic_module)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  baxter_core_msgs
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  pcl
  pcl_ros
  std_msgs
  tf
  visualization_msgs
  cv_bridge
  message_generation
)
find_package(GLUT REQUIRED)
find_package(OpenGL REQUIRED)
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
   Deproject.srv
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
  std_msgs  # and/or other packages containing depended messages
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
  INCLUDE_DIRS
  LIBRARIES display
  CATKIN_DEPENDS baxter_core_msgs geometry_msgs nav_msgs roscpp rospy std_msgs tf
  DEPENDS system_lib
)

###########
## Build ##
###########
## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenGL_INCLUDE_DIRS}
  ${GLUT_INCLUDE_DIRS}
)

## Add subdirectory
add_subdirectory(src)

## Declare a cpp library
# add_library(display
#   sample/${PROJECT_NAME}/display.cpp 
# )

#set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
set(CMAKE_BUILD_TYPE Release)
add_definitions(-std=c++0x)

add_executable(sample_hook sample/sample_hook.cpp
)
target_link_libraries(sample_hook
  ros_module
  ${OPENGL_LIBRARIES}
  ${OpenCV_LIBS}
  ${GLUT_LIBRARY}
  ${catkin_LIBRARIES}
)
add_executable(sample_arm sample/sample_arm.cpp
)
target_link_libraries(sample_arm
  ros_module
  ${OPENGL_LIBRARIES}
  ${OpenCV_LIBS}
  ${GLUT_LIBRARY}
  ${catkin_LIBRARIES}
)
add_executable(sample_ik sample/sample_ik.cpp
)
target_link_libraries(sample_ik
  ros_module
  ${OPENGL_LIBRARIES}
  ${OpenCV_LIBS}
  ${GLUT_LIBRARY}
  ${catkin_LIBRARIES}
)
add_executable(sample_stop sample/sample_stop.cpp
)
target_link_libraries(sample_stop
  ros_module
  ${OPENGL_LIBRARIES}
  ${OpenCV_LIBS}
  ${GLUT_LIBRARY}
  ${catkin_LIBRARIES}
)
add_executable(sample_led sample/sample_led.cpp
)
target_link_libraries(sample_led
  ros_module
  ${catkin_LIBRARIES}
)
add_executable(sample_tf_listen sample/sample_tf_listen.cpp
)
target_link_libraries(sample_tf_listen
  ros_module
  ${catkin_LIBRARIES}
)
add_executable(sample_tf_broadcast sample/sample_tf_broadcast.cpp
)
target_link_libraries(sample_tf_broadcast
  ros_module
  ${catkin_LIBRARIES}
)
add_executable(sample_camera_controll sample/sample_camera_controll.cpp
)
target_link_libraries(sample_camera_controll
  ros_module
  ${catkin_LIBRARIES}
)
add_executable(sample_subscribe sample/sample_subscribe.cpp
)
target_link_libraries(sample_subscribe
  ros_module
  ${catkin_LIBRARIES}
  ${GLUT_LIBRARY}
)
add_executable(sample_realsense sample/sample_realsense.cpp
)
target_link_libraries(sample_realsense
  ros_module
  ${catkin_LIBRARIES}
  ${GLUT_LIBRARY}
)
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
# install(TARGETS display display_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
 install(DIRECTORY include/ros_module/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.hpp"
#   PATTERN ".svn" EXCLUDE
 )

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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_display.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
