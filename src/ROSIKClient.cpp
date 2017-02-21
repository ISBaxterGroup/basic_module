/**
 * @file ROSIKClient.cpp
 * @brief 
 *	Implimentation of ROSIKClient
 * @author Iwase
 */
//----------------------------------------------------------
// Include
//----------------------------------------------------------
#include <ros_module/ROSInterface.hpp>
//----------------------------------------------------------
// Constant parameter
//----------------------------------------------------------
const int ROSIKClient::SEED_AUTO(0);
const int ROSIKClient::SEED_USER(1);
const int ROSIKClient::SEED_CURRENT(2);
const int ROSIKClient::SEED_NS_MAP(3);

// Names in arm joints
const std::array<std::string, ROSIKClient::JOINTS_NUM> ROSIKClient::LEFT_JOINT_NAMES = {"left_s0", "left_s1", "left_e0", "left_e1", "left_w0", "left_w1", "left_w2"};
const std::array<std::string, ROSIKClient::JOINTS_NUM> ROSIKClient::RIGHT_JOINT_NAMES = {"right_s0", "right_s1", "right_e0", "right_e1", "right_w0", "right_w1", "right_w2"};

//----------------------------------------------------------
// LRRequest
//----------------------------------------------------------
ROSIKClient::LRRequest::LRRequest(const BCMSSolvePositionIK& r) : BCMSSolvePositionIK(r) {};
ROSIKClient::LRRequest::LRRequest(const int mode, const std::array<double, 3>& x, const std::array<double, 4>& q)
{
	assert(mode != SEED_USER);

	request.pose_stamp.resize(1);
	request.pose_stamp[0].header.stamp = ros::Time::now();
	request.pose_stamp[0].header.frame_id = mode;
	request.pose_stamp[0].pose.position.x = x[0];
	request.pose_stamp[0].pose.position.y = x[1];
	request.pose_stamp[0].pose.position.z = x[2];
	request.pose_stamp[0].pose.orientation.x = q[0];
	request.pose_stamp[0].pose.orientation.y = q[1];
	request.pose_stamp[0].pose.orientation.z = q[2];
	request.pose_stamp[0].pose.orientation.w = q[3];

	request.seed_mode = mode;
};
ROSIKClient::LRRequest::LRRequest(const int mode, const std::array<double, 7>& seed_angles, const std::array<double, 3>& x, const std::array<double, 4>& q)
{

	request.pose_stamp.resize(1);
	request.pose_stamp[0].header.stamp = ros::Time::now();
	request.pose_stamp[0].header.frame_id = mode;
	request.pose_stamp[0].pose.position.x = x[0];
	request.pose_stamp[0].pose.position.y = x[1];
	request.pose_stamp[0].pose.position.z = x[2];
	request.pose_stamp[0].pose.orientation.x = q[0];
	request.pose_stamp[0].pose.orientation.y = q[1];
	request.pose_stamp[0].pose.orientation.z = q[2];
	request.pose_stamp[0].pose.orientation.w = q[3];

	request.seed_angles.resize(1);
	request.seed_angles[0].position.resize(JOINTS_NUM);
	copy_n(seed_angles.begin(), JOINTS_NUM, request.seed_angles[0].position.begin());

	request.seed_mode = mode;

};
//----------------------------------------------------------
// LeftRequest
//----------------------------------------------------------
ROSIKClient::LeftRequest::LeftRequest(const BCMSSolvePositionIK& r) : ROSIKClient::LRRequest(r){};
ROSIKClient::LeftRequest::LeftRequest(const int mode, const std::array<double, 3>& x, const std::array<double, 4>& q) : ROSIKClient::LRRequest(mode, x, q){};
ROSIKClient::LeftRequest::LeftRequest(const int mode, const std::array<double, 7>& seed_angles, const std::array<double, 3>& x, const std::array<double, 4>& q) 
	: ROSIKClient::LRRequest(mode, seed_angles, x, q)
{
	request.seed_angles[0].name.resize(JOINTS_NUM);
	copy_n(LEFT_JOINT_NAMES.begin(), JOINTS_NUM, request.seed_angles[0].name.begin());
};

//----------------------------------------------------------
// RightRequest
//----------------------------------------------------------
ROSIKClient::RightRequest::RightRequest(const BCMSSolvePositionIK& r) : ROSIKClient::LRRequest(r){};
ROSIKClient::RightRequest::RightRequest(const int mode, const std::array<double, 3>& x, const std::array<double, 4>& q) : ROSIKClient::LRRequest(mode, x, q){};
ROSIKClient::RightRequest::RightRequest(const int mode, const std::array<double, 7>& seed_angles, const std::array<double, 3>& x, const std::array<double, 4>& q) 
	: ROSIKClient::LRRequest(mode, seed_angles, x, q)
{	
	request.seed_angles[0].name.resize(JOINTS_NUM);
	copy_n(RIGHT_JOINT_NAMES.begin(), JOINTS_NUM, request.seed_angles[0].name.begin());
};

//----------------------------------------------------------
// ROSIKClient
//----------------------------------------------------------
ROSIKClient::ROSIKClient()
{};
void ROSIKClient::init()
{
	ros::NodeHandle n;
    left_client = n.serviceClient<baxter_core_msgs::SolvePositionIK>("ExternalTools/left/PositionKinematicsNode/IKService");
    right_client = n.serviceClient<baxter_core_msgs::SolvePositionIK>("ExternalTools/right/PositionKinematicsNode/IKService");
};
bool ROSIKClient::call(LeftRequest& r)
{
	baxter_core_msgs::SolvePositionIK* r_ptr = &r;
	if (!left_client.call(*r_ptr))
	    ROS_ERROR("rf::search_angle - FAILED to left call service");
	if (r.response.isValid[0])
		copy_n(r.response.joints[0].position.begin(), JOINTS_NUM, left_joints.begin());
	
	return r.response.isValid[0];
};
bool ROSIKClient::call(RightRequest& r)
{
	baxter_core_msgs::SolvePositionIK* r_ptr = &r;
	if (!right_client.call(*r_ptr))
	    ROS_ERROR("rf::search_angle - FAILED to right call service");
	if (r.response.isValid[0])
		copy_n(r.response.joints[0].position.begin(), JOINTS_NUM, right_joints.begin());
	
	return r.response.isValid[0];
};
std::array<double, ROSIKClient::JOINTS_NUM> ROSIKClient::get_left_joints() const { return left_joints; };
std::array<double, ROSIKClient::JOINTS_NUM> ROSIKClient::get_right_joints() const { return right_joints; };