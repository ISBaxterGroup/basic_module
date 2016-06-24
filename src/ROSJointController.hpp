//------------------------------------------------------------
// #Description : ROSJointController
//  This class publishes angle in each joint.
// Angles can be set by using the class of Left/RightJCommand
// #author: Iwase Hajime
// #date : 2016.06.01.
//------------------------------------------------------------
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
#include <baxter_core_msgs/JointCommand.h>

//----------------------------------------------------------
// Forward Declaration
//----------------------------------------------------------
typedef std_msgs::Int32 SMInt32;
typedef std_msgs::UInt16 SMUInt16;
typedef std_msgs::Float64 SMFloat64;
typedef baxter_core_msgs::JointCommand BCMJointCommand;

struct LRJCommand;
struct LeftJCommand;
struct RightJCommand;

//----------------------------------------------------------
// ROSJointController
//----------------------------------------------------------
class ROSJointController{
public:
	static constexpr unsigned int JOINTS_NUM = 7;	// The number of joints in arm

public:
	struct LRJCommand : public BCMJointCommand{
		explicit LRJCommand(const BCMJointCommand &);
		LRJCommand(const std::array<double, JOINTS_NUM> &);
	};
	struct LeftJCommand : LRJCommand {
		explicit LeftJCommand(const BCMJointCommand &);
		LeftJCommand(const std::array<double, JOINTS_NUM> &);
	};
	struct RightJCommand : LRJCommand {
		explicit RightJCommand(const BCMJointCommand &);
		RightJCommand(const std::array<double, JOINTS_NUM> &);
	};

public:

	static const unsigned int POSITION_MODE; 	// Joint controll mode
	static const unsigned int PUBLISH_FREQUENCY;
	static const double SPEED_RATIO;	// The move speed of motor 0.0 - 1.0 (default 0.3)  
	static const double CMD_TIMEOUT;	// Time out until controll mode in baxter will losts

	static const std::array<std::string, JOINTS_NUM> LEFT_JOINT_NAMES;
	static const std::array<std::string, JOINTS_NUM> RIGHT_JOINT_NAMES;

	static const std::array<double, JOINTS_NUM> INIT_LEFT_JOINT_COMMAND;
	static const std::array<double, JOINTS_NUM> INIT_RIGHT_JOINT_COMMAND;

private:
	bool exit;

	std::mutex mtx;
	std::thread controll_thread;

	ros::Publisher pub_rate;
	ros::Publisher pub_left_speed_ratio, pub_left_cmd, pub_left_cmd_timeout;
	ros::Publisher pub_right_speed_ratio, pub_right_cmd, pub_right_cmd_timeout;

	BCMJointCommand left_joint_cmd, right_joint_cmd;
	
public:
	ROSJointController();
	~ROSJointController();

	void init();

	void set_command(const LeftJCommand &);
	void set_command(const RightJCommand &);

private:
	void publish_loop();

};

typedef std::array<double, ROSJointController::JOINTS_NUM> Joints;
#endif