/**
 * @file ROSHookController.cpp
 * @brief 
 *	Implimentation of ROSHookController
 * @author Iwase
 */
//----------------------------------------------------------
// Include
//----------------------------------------------------------
#include "ROSHookController.hpp"
//----------------------------------------------------------
// Constant parameter
//----------------------------------------------------------
const std::string ROSHookController::MSG_LEFT_OPEN("h");
const std::string ROSHookController::MSG_RIGHT_OPEN("g");
const std::string ROSHookController::MSG_BOTH_OPEN("e");
const std::string ROSHookController::MSG_BOTH_CLOSE("f");

//----------------------------------------------------------
// ROSHookController
//----------------------------------------------------------
ROSHookController::ROSHookController():
	left_state(INIT_LEFT_STATE),
	right_state(INIT_RIGHT_STATE),
	init_flag(false)
{};
void ROSHookController::init()
{
	ros::NodeHandle n;
	// Publish rate publisher
	publisher = n.advertise<std_msgs::String>("hook", 10);
	init_flag = true;
	publish();
};
void ROSHookController::set_command(const E_Left_State& c)
{
	assert(init_flag);
	if(left_state != c){
		left_state = c;
		publish();
	}
};
void ROSHookController::set_command(const E_Right_State& c)
{
	assert(init_flag);
	if(right_state != c){
		right_state = c;
		publish();
	}
};
void ROSHookController::set_command(const E_Left_State& c1, const E_Right_State& c2)
{
	assert(init_flag);
	if(left_state != c1 || right_state != c2){
		left_state = c1;
		right_state = c2;
		publish();
	}
};
void ROSHookController::left_reverce()
{
	assert(init_flag);
	switch(left_state){
		case E_Left_State::E_Open:
			left_state = E_Left_State::E_Close;
			break;
		case E_Left_State::E_Close:
			left_state = E_Left_State::E_Open;
			break;
		default:
			break;
	}
	publish();
};
void ROSHookController::right_reverce()
{
	assert(init_flag);
	switch(right_state){
		case E_Right_State::E_Open:
			right_state = E_Right_State::E_Close;
			break;
		case E_Right_State::E_Close:
			right_state = E_Right_State::E_Open;
			break;
		default:
			break;
	}
	publish();
};
void ROSHookController::publish()
{
	assert(init_flag);
	std_msgs::String msg;
	if(left_state == E_Left_State::E_Open && right_state == E_Right_State::E_Open){
		msg.data = MSG_BOTH_OPEN;
	}
	else if(left_state == E_Left_State::E_Close && right_state == E_Right_State::E_Close){
		msg.data = MSG_BOTH_CLOSE;
	}
	else if(left_state == E_Left_State::E_Open && right_state == E_Right_State::E_Close){
		msg.data = MSG_LEFT_OPEN;
	}
	else if(left_state == E_Left_State::E_Close && right_state == E_Right_State::E_Open){
		msg.data = MSG_RIGHT_OPEN;
	}
	publisher.publish(msg);
};

