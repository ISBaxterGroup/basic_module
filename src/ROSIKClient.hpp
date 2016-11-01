/**
 * @file ROSIKClient.hpp
 * @brief 
 *	Class for calling ik service easily.
 * @author Iwase
 * @date 2016.06.01.
 */
#ifndef ROS_IK_CLIENT_HPP
#define ROS_IK_CLIENT_HPP

#include <iostream>
#include <array>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <baxter_core_msgs/SolvePositionIK.h>

typedef std_msgs::String SMString;
typedef baxter_core_msgs::SolvePositionIK BCMSSolvePositionIK;
/**
* @class ROSIKClient
* @brief An interface with Ikservice.
*/
class ROSIKClient{
private:
	struct LRRequest : public BCMSSolvePositionIK{
		explicit LRRequest(const BCMSSolvePositionIK&);
		LRRequest(const int, const std::array<double, 3>&, const std::array<double, 4>&);
		LRRequest(const int, const std::array<double, 7>&, const std::array<double, 3>&, const std::array<double, 4>&);
	};
public:
	//! The number of joints in arm
	static constexpr unsigned int JOINTS_NUM = 7;	
	/**
	* @class LeftRequest
	* @brief Wrapper class to describe Left hand state
	*/
	struct LeftRequest : LRRequest{
		explicit LeftRequest(const BCMSSolvePositionIK&);
		/**
		 * @brief A constructor
	 	 * @param seed_mode (see http://sdk.rethinkrobotics.com/wiki/API_Reference)
	 	 * @param trans target translation
	 	 * @param rot target rotation
		 */
		LeftRequest(const int seed_mode, const std::array<double, 3>& trans, const std::array<double, 4>& rot);
		/**
		 * @brief A constructor
	 	 * @param seed_mode (see http://sdk.rethinkrobotics.com/wiki/API_Reference)
	 	 * @param seed_angle seed angle
	 	 * @param trans target translation
	 	 * @param rot target rotation
		 */
		LeftRequest(const int seed_mode, const std::array<double, 7>& seed_angle, const std::array<double, 3>& trans, const std::array<double, 4>& rot);
	};
	/**
	* @class RightRequest
	* @brief Wrapper class to describe Right hand state.
	*/
	struct RightRequest : LRRequest{
		explicit RightRequest(const BCMSSolvePositionIK&);
		/**
		 * @brief A constructor
	 	 * @param seed_mode (see http://sdk.rethinkrobotics.com/wiki/API_Reference)
	 	 * @param trans target translation
	 	 * @param rot target rotation
		 */
		RightRequest(const int seed_mode, const std::array<double, 3>& trans, const std::array<double, 4>& rot);
		/**
		 * @brief A constructor
	 	 * @param seed_mode (see http://sdk.rethinkrobotics.com/wiki/API_Reference)
	 	 * @param seed_angle seed angle
	 	 * @param trans target translation
	 	 * @param rot target rotation
		 */
		RightRequest(const int seed_mode, const std::array<double, 7>& seed_angle, const std::array<double, 3>& trans, const std::array<double, 4>& rot);
	};

public:
	static const int SEED_AUTO;
	static const int SEED_USER;
	static const int SEED_CURRENT;
	static const int SEED_NS_MAP;

private:
	static const std::array<std::string, JOINTS_NUM> LEFT_JOINT_NAMES;
	static const std::array<std::string, JOINTS_NUM> RIGHT_JOINT_NAMES;

private:
	bool init_flag;
	ros::ServiceClient left_client;
	ros::ServiceClient right_client;

	std::array<double, JOINTS_NUM> left_joints;
	std::array<double, JOINTS_NUM> right_joints;

public:
	/**
	 * @brief A constructor 
	 */
	ROSIKClient();
	/**
	 * @brief Initialize publisher and start a thread
	 */
	void init();
	/**
	 * @brief Call IK service
	 * The angles that solved by IK service will be stored in this class.
	 * @param right_state The state of left gripper.
	 * @return Whether IK has been solved (true : solved)
	 */
	bool call(LeftRequest& left_state);
	/**
	 * @brief Call IK service
	 * The angles that solved by IK service will be stored in this class.
	 * @param right_state The state of left gripper.
	 * @return Whether IK has been solved (true : solved)
	 */
	bool call(RightRequest& left_state);
	/**
	 * @brief get angles that solved by IK
	 * @return Angle array
	 */
	std::array<double, JOINTS_NUM> get_left_joints() const;
	/**
	 * @brief get angles that solved by IK
	 * @return Angle array
	 */
	std::array<double, JOINTS_NUM> get_right_joints() const;
	
}; 

#endif