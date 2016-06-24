//----------------------------------------------------------
// Include
//----------------------------------------------------------
#include "ROSIKClient.hpp"

const int ROSIKClient::SEED_AUTO(0);
const int ROSIKClient::SEED_USER(1);
const int ROSIKClient::SEED_CURRENT(2);
const int ROSIKClient::SEED_NS_MAP(3);
//----------------------------------------------------------
// Constant parameter
//----------------------------------------------------------
ROSIKClient::LRRequest::LRRequest(const BCMSSolvePositionIK& r) : BCMSSolvePositionIK(r) {};
ROSIKClient::LRRequest::LRRequest(const int mode, const std::array<double, 3>& x, const std::array<double, 4>& q)
{
	assert(x.size() == 3);
	assert(q.size() == 4);

	request.pose_stamp.resize(1);
	request.pose_stamp[0].header.stamp = ros::Time::now();
	request.pose_stamp[0].header.frame_id = "base";
	request.pose_stamp[0].pose.position.x = x[0];
	request.pose_stamp[0].pose.position.y = x[1];
	request.pose_stamp[0].pose.position.z = x[2];
	request.pose_stamp[0].pose.orientation.x = q[0];
	request.pose_stamp[0].pose.orientation.y = q[1];
	request.pose_stamp[0].pose.orientation.z = q[2];
	request.pose_stamp[0].pose.orientation.w = q[3];

	request.seed_mode = mode;
};

ROSIKClient::LeftRequest::LeftRequest(const BCMSSolvePositionIK& r) : ROSIKClient::LRRequest(r){};
ROSIKClient::LeftRequest::LeftRequest(const int mode, const std::array<double, 3>& x, const std::array<double, 4>& q) : ROSIKClient::LRRequest(mode, x, q){};

ROSIKClient::RightRequest::RightRequest(const BCMSSolvePositionIK& r) : ROSIKClient::LRRequest(r){};
ROSIKClient::RightRequest::RightRequest(const int mode, const std::array<double, 3>& x, const std::array<double, 4>& q) : ROSIKClient::LRRequest(mode, x, q){};

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
	if (r.response.isValid[0]){
		copy_n(r.response.joints[0].position.begin(), JOINTS_NUM, left_joints.begin());
	}
	return r.response.isValid[0];
};
bool ROSIKClient::call(RightRequest& r)
{
	baxter_core_msgs::SolvePositionIK* r_ptr = &r;
	if (!right_client.call(*r_ptr))
	    ROS_ERROR("rf::search_angle - FAILED to right call service");
	if (r.response.isValid[0]){
		copy_n(r.response.joints[0].position.begin(), JOINTS_NUM, right_joints.begin());
	}
	return r.response.isValid[0];
};
std::array<double, ROSIKClient::JOINTS_NUM> ROSIKClient::get_left_joints() const { return left_joints; };
std::array<double, ROSIKClient::JOINTS_NUM> ROSIKClient::get_right_joints() const { return right_joints; };