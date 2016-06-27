//------------------------------------------------------------
// #Description : ROSIKClient
// Class for calling ik service easily.
// #author: Iwase Hajime
// #date : 2016.06.01.
//------------------------------------------------------------
#ifndef ROS_IK_CLIENT_HPP
#define ROS_IK_CLIENT_HPP

#include <iostream>
#include <array>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <baxter_core_msgs/SolvePositionIK.h>

typedef std_msgs::String SMString;
typedef baxter_core_msgs::SolvePositionIK BCMSSolvePositionIK;

//----------------------------------------------------------
// ROSIKClient
//----------------------------------------------------------
class ROSIKClient{
public:

	static constexpr unsigned int JOINTS_NUM = 7;	// The number of joints in arm

	struct LRRequest : public BCMSSolvePositionIK{
		explicit LRRequest(const BCMSSolvePositionIK&);
		LRRequest(const int, const std::array<double, 3>&, const std::array<double, 4>&);
		LRRequest(const int, const std::array<double, 7>&, const std::array<double, 3>&, const std::array<double, 4>&);
	};
	struct LeftRequest : LRRequest{
		explicit LeftRequest(const BCMSSolvePositionIK&);
		LeftRequest(const int, const std::array<double, 3>&, const std::array<double, 4>&);
		LeftRequest(const int, const std::array<double, 7>&, const std::array<double, 3>&, const std::array<double, 4>&);
	};
	struct RightRequest : LRRequest{
		explicit RightRequest(const BCMSSolvePositionIK&);
		RightRequest(const int, const std::array<double, 3>&, const std::array<double, 4>&);
		RightRequest(const int, const std::array<double, 7>&, const std::array<double, 3>&, const std::array<double, 4>&);
	};

	static const int SEED_AUTO;
	static const int SEED_USER;
	static const int SEED_CURRENT;
	static const int SEED_NS_MAP;

	static const std::array<std::string, JOINTS_NUM> LEFT_JOINT_NAMES;
	static const std::array<std::string, JOINTS_NUM> RIGHT_JOINT_NAMES;

private:
	bool init_flag;
	ros::ServiceClient left_client;
	ros::ServiceClient right_client;

	std::array<double, JOINTS_NUM> left_joints;
	std::array<double, JOINTS_NUM> right_joints;

public:
	ROSIKClient();

	void init();

	bool call(LeftRequest& );
	bool call(RightRequest& );

	std::array<double, JOINTS_NUM> get_left_joints() const;
	std::array<double, JOINTS_NUM> get_right_joints() const;
}; 

#endif