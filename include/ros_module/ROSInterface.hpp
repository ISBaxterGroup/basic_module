/**
 * @file ROSInterface.hpp
 * @brief Interface Library for easy access to the ROS
 * @author Iwase
 * @date 2016.9.26.
 * @mainpage 
 * - This project consists of roughly 7 modules. ROSInterface.hpp is including all hedders.
 * -# ROSFramework.hpp
 * -# ROSHookController.hpp
 * -# ROSJointController.hpp
 * -# ROSIKClient.hpp
 * -# ROSTFListener.hpp
 * -# ROSTFBroadcaster.hpp
 * -# Others
 *
 * @section main_sec1 Using Basic functions of ros.
 * - @ref ROSSubscriberInterface< T >
 * 	- sample program : @ref sample_subscribe.cpp
 * - @ref ROSServiceInterface< T >
 * 	- sample program : ROSCameraController.cpp
 * @section main_sec2 Using gripper.
 * - @ref ROSHookController
 *  - sample program : sample_hook.cpp
 * @section main_sec3 Using baxter arm.
 * - @ref ROSJointController
 *  - sample program : sample_arm.cpp
 * @section main_sec4 Using ik solver.
 * - @ref ROSIKClient
 *  - sample program : sample_ik.cpp
 * @section main_sec5 Using tf.
 * - @ref ROSTFListener
 *  - sample program : sample_tf_listen.cpp
 * - @ref ROSTFBroadcaster
 *  - sample program : sample_tf_broadcast.cpp
 * @section main_sec6 The others.
 * - @ref ROSCameraController
 *  - sample program : sample_camera_controll.cpp
 * - @ref ROSLEDController
 *  - sample program : sample_led.cpp
 */

#ifndef ROS_INTERFACE_HPP
#define ROS_INTERFACE_HPP

#include <ros/ros.h>

#include <ros_module/ROSFramework.hpp>
#include <ros_module/ROSHookController.hpp>
#include <ros_module/ROSJointController.hpp>
#include <ros_module/ROSIKClient.hpp>
#include <ros_module/ROSTFListener.hpp>
#include <ros_module/ROSTFBroadcaster.hpp>
#include <ros_module/ROSCameraController.hpp>
#include <ros_module/ROSLEDController.hpp>

#endif