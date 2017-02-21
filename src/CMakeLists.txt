cmake_minimum_required(VERSION 2.8.3)
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
###########
## Build ##
###########
## Declare a cpp library
add_definitions(-std=c++0x)
#set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
#add_definitions(-std=c++11)

add_library(ros_module STATIC
	ROSHookController.cpp
	ROSIKClient.cpp
	ROSJointController.cpp
	ROSTFBroadcaster.cpp
	ROSTFListener.cpp
	ROSLEDController.cpp
	ROSCameraController.cpp
)


