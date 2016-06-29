//------------------------------------------------------------
// #Description : ROSLEDController
// Class for access to Baxter io easily.
// #author: Iwase Hajime
// #date : 2016.06.01.
//------------------------------------------------------------
#ifndef ROS_LED_CONTROLLER_HPP
#define ROS_LED_CONTROLLER_HPP

#include <iostream>
#include <mutex>
#include <thread>
#include <array>
#include <ros/ros.h>
#include <std_msgs/UInt16.h>

typedef std_msgs::UInt16 SMUint16;
//----------------------------------------------------------
// ROSLEDController
//----------------------------------------------------------
class ROSLEDController{
public:
	static constexpr int LED_NUM = 12;
	static const unsigned int PUBLISH_FREQUENCY;

   	static const unsigned int DEFAULT_BEHAVIOR;
   	static const unsigned int OVERRIDE_ENABLE;

private:
	bool control_enable;
	bool thread_exit;
	std::mutex mtx;
	std::thread controll_thread;

	ros::Publisher publisher;

	std::array<bool, LED_NUM> state;

public:
	ROSLEDController();
	~ROSLEDController();
	// Initialize publisher and start a thread
	void init();
	// Enable controll
	void enable();
	// Disable controll
	void disable();
	// Set Led state
	void set_command(const std::array<bool, LED_NUM>&);

private:
	void publish_loop();
	SMUint16 to_message(const std::array<bool, LED_NUM>&) const;
};

#endif