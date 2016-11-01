/**
 * @file ROSTFListener.hpp
 * @brief 
 *	This class listenes the value of tf in each cycle. 
 *  And you can use tf easily.
 * @author Iwase
 * @date 2016.9.28.
 */
//----------------------------------------------------------
// Joint controller
//----------------------------------------------------------
#ifndef ROS_TF_LISTENER_HPP
#define ROS_TF_LISTENER_HPP

//----------------------------------------------------------
// include
//----------------------------------------------------------
#include <iostream>
#include <string>
#include <mutex>
#include <thread>
#include <array>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

/**
* @class ROSTFListener
* @brief An interface with TF.
*/
class ROSTFListener{
public:
	//! frequency
	static constexpr unsigned int DEFAULT_FREQUENCY = 20; 
	//! maximum delay [sec]
	static constexpr double MAXIMUM_DELAY = 1.0; 

private:
	bool exit;
	bool valid;

	unsigned int freq;

	std::string src_name;
	std::string dst_name;

	std::array<double, 3> curr_trans;
	std::array<double, 4> curr_rot;

	std::mutex mtx;
	std::thread listen_thread;

public:
	/**
	 * @brief constructor from two coordinate system
	 * @param src sorce coordinate system
	 * @param dst destination coordinate system
	 */
	ROSTFListener(const std::string& src, const std::string& dst);
	~ROSTFListener();

	/**
	 * @brief Start the listen loop thread
	 */
	void init();
	/**
	 * @brief setter
	 * @param src sorce coordinate system
	 */
	void set_source(const std::string& src);
	/**
	 * @brief setter
	 * @param dst destination coordinate system
	 */
	void set_destination(const std::string& dst);
	/**
	 * @brief setter
	 * @param src sorce coordinate system
	 * @param dst destination coordinate system
	 */
	void set_transform(const std::string& src, const std::string& dst);
	/**
	 * @brief getter
	 * @param trans translation src to dst.
	 * @param rot rotation src to dst.
	 * @return Is transform is valid value.
	 */
	bool get_transform(std::array<double, 3>& trans, std::array<double, 4>& rot);

private:
	void listen_loop();

};

#endif