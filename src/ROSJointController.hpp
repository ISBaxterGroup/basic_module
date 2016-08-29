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
#include <std_msgs/Empty.h>
#include <baxter_core_msgs/JointCommand.h>

//----------------------------------------------------------
// typedef
//----------------------------------------------------------
typedef baxter_core_msgs::JointCommand BCMJointCommand;

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
	bool enable_collision_avoidance;

	std::mutex mtx;
	std::thread controll_thread;

	ros::Publisher pub_rate;
	ros::Publisher pub_left_speed_ratio, pub_right_speed_ratio;
	ros::Publisher pub_left_cmd, pub_right_cmd;
	ros::Publisher pub_left_cmd_timeout, pub_right_cmd_timeout;
	ros::Publisher pub_left_stop_collision_avoidance, pub_right_stop_collision_avoidance;

	BCMJointCommand left_joint_cmd, right_joint_cmd;
	
public:
	ROSJointController();
	~ROSJointController();

	//----------------------------------------------------------
	// init
	//		Start the controle loop thread
	// Parameter
	//----------------------------------------------------------
	void init();
	//----------------------------------------------------------
	// collision_avoidance
	//		Change state of collision avoidance
	// Parameter
	// 		enable : Disable the collision_avoidance 
	//				 if you specify "false" while "true" will set.
	//----------------------------------------------------------
	void collision_avoidance(const bool enable);
	//----------------------------------------------------------
	// set command
	//		Set the goal of the joint angle.
	//		Joint angle specified by <Left/Right>JCommand class.
	//		<Left/Right>JCommand can be initialized by std::array<double, JOINTS_NUM>
	// Parameter
	// 		left_joint_angle : left joints 
	//		right_joint_angle: right joints
	//----------------------------------------------------------
	void set_command(const LeftJCommand & left_joint_angle);
	void set_command(const RightJCommand & right_joint_angle);

private:
	void publish_loop();

};

typedef std::array<double, ROSJointController::JOINTS_NUM> Joints;
#endif