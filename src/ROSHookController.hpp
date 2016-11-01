/**
 * @file ROSHookController.hpp
 * @brief 
 *	This class publishes gripper state.
 * State can be set by using the class of E_LEFT/RIGHT_State 
 * @author Iwase
 * @date 2016.06.01.
 */
#ifndef ROS_HOOK_CONTROLLER_HPP
#define ROS_HOOK_CONTROLLER_HPP

#include <iostream>
#include <string>
#include <ros/ros.h>
#include <std_msgs/String.h>

typedef std_msgs::String SMString;
/**
* @class ROSHookController
* @brief An interface with gripper
*/
class ROSHookController{
public:
	enum struct E_Left_State{E_Open, E_Close};
	enum struct E_Right_State{E_Open, E_Close};

private:
	static constexpr E_Left_State INIT_LEFT_STATE = E_Left_State::E_Open;
	static constexpr E_Right_State INIT_RIGHT_STATE = E_Right_State::E_Open;

	static const std::string MSG_LEFT_OPEN;
	static const std::string MSG_RIGHT_OPEN;
	static const std::string MSG_BOTH_OPEN;
	static const std::string MSG_BOTH_CLOSE;

	bool init_flag;
	
	E_Left_State left_state;
	E_Right_State right_state;

	ros::Publisher publisher;
public:
	/**
	 * @brief A constructor 
	 */
	ROSHookController();
	/**
	 * @brief Initialize publisher and start a thread
	 */
	void init();
	/**
	 * @brief Reverce left state (Open <-> Close)
	 */
	void left_reverce();
	/**
	 * @brief Reverce right state (Open <-> Close)
	 */
	void right_reverce();
	/**
	 * @brief setter
	 * @param state Gripper state (E_Open or E_Close)
	 */
	void set_command(const E_Left_State & state);
	/**
	 * @brief setter
	 * @param state Gripper state (E_Open or E_Close)
	 */
	void set_command(const E_Right_State & state);
	/**
	 * @brief setter
	 * @param left_state Gripper state (E_Open or E_Close)
	 * @param right_state Gripper state (E_Open or E_Close)
	 */
	void set_command(const E_Left_State & left_state, const E_Right_State & right_state);

private:
	void publish();
};

#endif