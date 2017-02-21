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
	bool initialized;

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
	 * @return Is transform is valid value.
	 */
	bool valid_data();
	/**
	 * @brief getter
	 * @param trans rotation src to dst.
	 * @return Is transform is valid value.
	 */
	bool get_translation(std::array<double, 3>& trans);
	bool get_translation(tf::Vector3& trans);
	/**
	 * @brief getter
	 * @param rot rotation src to dst.
	 * @return Is transform is valid value.
	 */
	bool get_rotation(std::array<double, 4>& rot);
	bool get_rotation(tf::Quaternion& rot);
	/**
	 * @brief getter
	 * @param trans translation src to dst.
	 * @param rot rotation src to dst.
	 * @return Is transform is valid value.
	 */
	bool get_transform(std::array<double, 3>& trans, std::array<double, 4>& rot);
	bool get_transform(tf::Vector3& trans, tf::Quaternion& rot);
	/**
	 * @brief transform point from src to dst
	 * @param pos position in src coordinate system
	 * @return position in dst coordinate system
	 */
	std::array<double, 3> transform(const std::array<double, 3>& pos);
	tf::Vector3 transform(const tf::Vector3& pos);
	tf::Quaternion transform(const tf::Quaternion& q);
	/**
	 * @brief inverse transform
	 * @param pos pos in dst coordinate system
	 * @return pos int src coordinate system
	 */
	std::array<double, 3> inverse_transform(const std::array<double, 3>& pos);
	tf::Vector3 inverse_transform(const tf::Vector3& pos);
	tf::Quaternion inverse_transform(const tf::Quaternion& q);
private:
	void listen_loop();

};

#endif