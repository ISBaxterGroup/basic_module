//------------------------------------------------------------
// #Description : ROSHookController
//  This class publishes gripper state.
// State can be set by using the class of E_LEFT/RIGHT_State 
// #author: Iwase Hajime
// #date : 2016.06.01.
//------------------------------------------------------------
#ifndef ROS_HOOK_CONTROLLER_HPP
#define ROS_HOOK_CONTROLLER_HPP

#include <iostream>
#include <string>
#include <ros/ros.h>
#include <std_msgs/String.h>

typedef std_msgs::String SMString;

//----------------------------------------------------------
// ROSHookController
//----------------------------------------------------------
class ROSHookController{
public:
	enum struct E_Left_State{E_Open, E_Close};
	enum struct E_Right_State{E_Open, E_Close};

	static constexpr E_Left_State INIT_LEFT_STATE = E_Left_State::E_Open;
	static constexpr E_Right_State INIT_RIGHT_STATE = E_Right_State::E_Open;

	static const std::string MSG_LEFT_OPEN;
	static const std::string MSG_RIGHT_OPEN;
	static const std::string MSG_BOTH_OPEN;
	static const std::string MSG_BOTH_CLOSE;

private:
	bool init_flag;
	
	E_Left_State left_state;
	E_Right_State right_state;

	ros::Publisher publisher;
public:
	ROSHookController();

	void init();

	void left_reverce();	// Reverce left state
	void right_reverce();	// Reverce left state

	void set_command(const E_Left_State &);
	void set_command(const E_Right_State &);
	void set_command(const E_Left_State &, const E_Right_State &);

private:
	void publish();
};

#endif