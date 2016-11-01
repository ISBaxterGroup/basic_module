/**
 * @file ROSTFBroadcaster.hpp
 * @brief 
 *	This class Broadcasts the tf value in each cycle. 
 * @author Iwase
 * @date 2016.9.29.
 */
//----------------------------------------------------------
// Joint controller
//----------------------------------------------------------
#ifndef ROS_TF_BROADCASTER_HPP
#define ROS_TF_BROADCASTER_HPP

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
* @class ROSTFBroadcaster
* @brief An interface with TF.
*/
class ROSTFBroadcaster{
public:
	//! frequency
	static constexpr unsigned int DEFAULT_FREQUENCY = 30; 

private:
	bool exit_;

	unsigned int freq_;

	std::string src_name_;
	std::string dst_name_;

	std::array<double, 3> trans_;
	std::array<double, 4> rot_;

	std::mutex mtx_;
	std::thread broadcast_thread_;

public:
	/**
	 * @brief A constructor
	 */
	ROSTFBroadcaster();
	/**
	 * @brief A constructor from coordinate system name
	 * @param src_name The name of transform sorce
	 * @param dst_name The name of transform destination
	 */
	ROSTFBroadcaster(const std::string& src_name, const std::string& dst_name);
	/**
	 * @brief A constructor from coordinate system state
	 * @param src_name The name of transform sorce
	 * @param dst_name The name of transform destination
	 * @param trans translation
	 * @param rot roation
	 */
	ROSTFBroadcaster(const std::string& src_name, const std::string& dst_name, const std::array<double, 3>& trans, const std::array<double, 4>& rot);
	~ROSTFBroadcaster();
	/**
	 * @brief Start the broadcast loop thread
	 */
	void init();
	/**
	 * @brief setter
	 * @param src_name The name of transform sorce
	 * @param dst_name The name of transform destination
	 */
	void set_name(const std::string& src_name, const std::string& dst_name);
	/**
	 * @brief setter
	 * @param trans translation
	 */
	void set_translation(const std::array<double, 3>& trans);
	/**
	 * @brief setter
	 * @param rot rotation
	 */
	void set_rotation(const std::array<double, 4>& rot);
	/**
	 * @brief getter
	 * @param trans translation
	 * @param rot roation
	 */
	bool set_transform(const std::array<double, 3>& trans, const std::array<double, 4>& rot);
	/**
	 * @brief setter 
	 * @param src_name The name of transform sorce
	 * @param dst_name The name of transform destination
	 * @param trans translation
	 * @param rot roation
	 */
	void set_transform(const std::string& src_name, const std::string& dst_name, const std::array<double, 3>& trans, const std::array<double, 4>& rot);

private:
	void broadcast_loop();

};

#endif