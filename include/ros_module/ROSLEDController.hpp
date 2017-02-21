/**
 * @file ROSLEDController.hpp
 * @brief 
 *	Class for access to Baxter io easily.
 * @author Iwase
 * @date 2016.06.01.
 */
#ifndef ROS_LED_CONTROLLER_HPP
#define ROS_LED_CONTROLLER_HPP

#include <iostream>
#include <mutex>
#include <thread>
#include <array>
#include <ros/ros.h>
#include <std_msgs/UInt16.h>

typedef std_msgs::UInt16 SMUint16;
/**
* @class ROSLEDController
* @brief An interface with LED in Baxter head.
*/
class ROSLEDController{
public:
	//! The number of LED on head
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
	/**
	 * @brief A constructor 
	 */
	ROSLEDController();
	~ROSLEDController();
	/**
	 * @brief Initialize publisher and start a thread
	 */
	void init();
	/**
	 * @brief Enable led controll
	 */
	void enable();
	/**
	 * @brief Disable led controll
	 */
	void disable();
	/**
	 * @brief setter
	 * @param led_state Led states array (true : ON, false : OFF)
	 */
	void set_command(const std::array<bool, LED_NUM>& led_state);

private:
	void publish_loop();
	SMUint16 to_message(const std::array<bool, LED_NUM>&) const;
};

#endif