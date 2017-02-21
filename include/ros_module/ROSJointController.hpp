/**
 * @file ROSJointController.hpp
 * @brief 
 *	This class publishes angle in each joint.
 *  Angles can be set by using the class of Left/RightJCommand
 * @author Iwase
 * @date 2016.06.01.
 */
//----------------------------------------------------------
// Joint controller
//----------------------------------------------------------
#ifndef ROS_JOINT_CONTROLLER_HPP
#define ROS_JOINT_CONTROLLER_HPP

//----------------------------------------------------------
// include
//----------------------------------------------------------
#include <iostream>
#include <string>
#include <mutex>
#include <thread>
#include <array>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/JointState.h>
#include <baxter_core_msgs/JointCommand.h>

//----------------------------------------------------------
// typedef
//----------------------------------------------------------
/**
* @class ROSJointController
* @brief An interface with Baxter Joints
*  Angles can be set by using the class of Left/RightJCommand.
*/
class ROSJointController{
public:
	//! The number of joints in arm
	static constexpr unsigned int JOINTS_NUM = 7;	
	//! Number of other joint
	static constexpr unsigned int OTHER_JOINTS_NUM = 3; 

private:
	struct LRJCommand : public baxter_core_msgs::JointCommand{
		explicit LRJCommand(const baxter_core_msgs::JointCommand &);
		LRJCommand(const std::array<double, JOINTS_NUM> &);
	};
	
public:
	/**
	* Derive the std::array in order to distinguish between the left and right.
	*/
	struct LeftJCommand : LRJCommand {
		explicit LeftJCommand(const baxter_core_msgs::JointCommand &);
		LeftJCommand(const std::array<double, JOINTS_NUM> &);
	};
	/**
	* Derive the std::array in order to distinguish between the left and right.
	*/
	struct RightJCommand : LRJCommand {
		explicit RightJCommand(const baxter_core_msgs::JointCommand &);
		RightJCommand(const std::array<double, JOINTS_NUM> &);
	};

private:
	//! Joint controll mode
	static const unsigned int POSITION_MODE; 	
	static const unsigned int PUBLISH_FREQUENCY;
	static const unsigned int SUBSCRIBE_FREQUENCY;
	//! The move speed of motor 0.0 - 1.0 (default 0.3)  
	static const double SPEED_RATIO;
	//! Time out until controll mode in baxter will losts
	static const double CMD_TIMEOUT;	
	static const std::array<std::string, JOINTS_NUM> LEFT_JOINT_NAMES;
	static const std::array<std::string, JOINTS_NUM> RIGHT_JOINT_NAMES;
	static const std::array<std::string, 3> OTHER_JOINT_NAMES;
	static const std::array<double, JOINTS_NUM> INIT_LEFT_JOINT_COMMAND;
	static const std::array<double, JOINTS_NUM> INIT_RIGHT_JOINT_COMMAND;

private:
	bool exit_pub_;
	bool exit_sub_;
	bool enable_collision_avoidance;

	std::mutex mtx_pub;
	std::mutex mtx_sub;
	std::thread controll_thread;
	std::thread subscribe_thread;

	ros::Publisher pub_rate;
	ros::Publisher pub_left_speed_ratio, pub_right_speed_ratio;
	ros::Publisher pub_left_cmd, pub_right_cmd;
	ros::Publisher pub_left_cmd_timeout, pub_right_cmd_timeout;
	ros::Publisher pub_left_stop_collision_avoidance, pub_right_stop_collision_avoidance;

	ros::Subscriber sub_joint_state;

	baxter_core_msgs::JointCommand left_joint_cmd, right_joint_cmd;
	sensor_msgs::JointState curr_joint_state;
public:
	/**
	 * @brief A constructor 
	 */
	ROSJointController();
	~ROSJointController();
	/**
	 * @brief Initialize publisher and start a thread
	 */
	void init();
	/**
	 * @brief Change state of collision avoidance
	 * @param enable Disable the collision_avoidance
	 * if you specify "false" while "true" will set.
	 */
	void collision_avoidance(const bool enable);	
	/**
	 * @brief setter
	 *		Joint angle specified by <Left/Right>JCommand class.
	 *		<Left/Right>JCommand can be initialized by std::array<double, JOINTS_NUM>
	 * @param left_joint_angle left joints states
	 */
	void set_command(const LeftJCommand & left_joint_angle);
	/**
	 * @brief setter.
	 *		Joint angle specified by <Left/Right>JCommand class.
	 *		<Left/Right>JCommand can be initialized by std::array<double, JOINTS_NUM>
	 * @param right_joint_angle right joints states
	 */
	void set_command(const RightJCommand & right_joint_angle);
	/**
	 * @brief getter.
	 *		Get current joint postion that subscribed from robot_state_publisher
	 * @param left_joint_angle left joints state
	 */
	void get_left_joint_angle(std::array<double, JOINTS_NUM> & left_joint_angle);
	/**
	 * @brief getter.
	 *		Get current joint postion that subscribed from robot_state_publisher
	 * @param left_joint_angle left joints state
	 */
	void get_right_joint_angle(std::array<double, JOINTS_NUM> & right_joint_angle);
	/**
	 * @brief getter.
	 *		Get current joint postion that subscribed from robot_state_publisher
	 * @param right_joint_angle right joints state
	 */
	void get_joint_angle(std::array<double, JOINTS_NUM> & left_joint_angle, std::array<double, JOINTS_NUM> & right_joint_angle);

private:
	void sub_state_callback(const sensor_msgs::JointState& joint_state);

private:
	void publish_loop();
	void subscribe_loop();

};

typedef std::array<double, ROSJointController::JOINTS_NUM> Joints;
#endif