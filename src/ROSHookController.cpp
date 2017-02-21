/**
 * @file ROSHookController.cpp
 * @brief 
 *	Implimentation of ROSHookController
 * @author Iwase
 */
//----------------------------------------------------------
// Include
//----------------------------------------------------------
#include <ros_module/ROSInterface.hpp>
//----------------------------------------------------------
// Constant parameter
//----------------------------------------------------------
const std::string ROSHookController::MSG_LEFT_OPEN("h");
const std::string ROSHookController::MSG_RIGHT_OPEN("g");
const std::string ROSHookController::MSG_BOTH_OPEN("e");
const std::string ROSHookController::MSG_BOTH_CLOSE("f");
const std::string ROSHookController::MSG_LEFT_ANGLE("l");
const std::string ROSHookController::MSG_RIGHT_ANGLE("r");

//----------------------------------------------------------
// Class Degree, Radian
//----------------------------------------------------------
ROSHookController::Degree::Degree(const double val) : Carrier(val) { assert(val >= 0.0 && val <= 90.0); };
ROSHookController::Degree::operator double() const { return val_; };
ROSHookController::Degree::operator Radian() const { return Radian( val_ * M_PI / 180.0);};

ROSHookController::Radian::Radian(const double val) : Carrier(val) { assert(val >= 0.0 && val <= M_PI_2); };
ROSHookController::Radian::operator double() const { return val_; };
ROSHookController::Radian::operator Degree() const { return Degree( val_ * 180.0 / M_PI);};

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
void ROSHookController::set_state(const E_Left_State& c)
{
	assert(init_flag);
	if(left_state != c){
		left_state = c;
		publish();
	}
};
void ROSHookController::set_state(const E_Right_State& c)
{
	assert(init_flag);
	if(right_state != c){
		right_state = c;
		publish();
	}
};
void ROSHookController::set_state(const E_Left_State& c1, const E_Right_State& c2)
{
	assert(init_flag);
	if(left_state != c1 || right_state != c2){
		left_state = c1;
		right_state = c2;
		publish();
	}
};

void ROSHookController::set_state(const E_Left_State & state, const Degree& angle)
{
	assert(init_flag);
	assert(state == E_Left_State::E_Angle);
	publish(state, (double)angle);
};
void ROSHookController::set_state(const E_Right_State & state, const Degree& angle)
{
	assert(init_flag);
	assert(state == E_Right_State::E_Angle);
	publish(state, (double)angle);
};
void ROSHookController::set_state(const E_Left_State & state, const Radian& angle)
{
	assert(init_flag);
	assert(state == E_Left_State::E_Angle);
	set_state(state, Degree(angle));
};
void ROSHookController::set_state(const E_Right_State & state, const Radian& angle)
{
	assert(init_flag);
	assert(state == E_Right_State::E_Angle);
	set_state(state, Degree(angle));
};
void ROSHookController::set_state(const Degree & left_angle, const Degree & right_angle)
{
	assert(init_flag);
	set_state(E_Left_State::E_Angle, left_angle);
	set_state(E_Right_State::E_Angle, right_angle);
};
void ROSHookController::set_state(const Radian & left_angle, const Radian & right_angle)
{
	assert(init_flag);
	set_state(E_Left_State::E_Angle, Degree(left_angle));
	set_state(E_Right_State::E_Angle, Degree(right_angle));
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
void ROSHookController::publish(const E_Left_State& state, const double angle)
{
	assert(init_flag);
	std::stringstream ss;
	ss << MSG_LEFT_ANGLE << " " << std::fixed << std::setprecision(OUTPUT_NUMBER_OF_DECIMAL_PLACES) << angle;
	
	std_msgs::String msg;
	msg.data = ss.str();
	publisher.publish(msg);
};
void ROSHookController::publish(const E_Right_State& state, const double angle)
{
	assert(init_flag);
	std::stringstream ss;
	ss << MSG_RIGHT_ANGLE << " " << std::fixed << std::setprecision(OUTPUT_NUMBER_OF_DECIMAL_PLACES) << angle;
	
	std_msgs::String msg;
	msg.data = ss.str();
	publisher.publish(msg);

};