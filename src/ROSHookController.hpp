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
#include <iomanip>
#include <string>
#include <sstream>
#include <ros/ros.h>
#include <std_msgs/String.h>

typedef std_msgs::String SMString;
/**
* @class ROSHookController
* @brief An interface with gripper
*/
class ROSHookController{
private:
	class Carrier{
	protected:
		double val_;

	public:
		Carrier();
		Carrier(const double val){ val_ = val; };
		operator double() const { return val_; };
	};
public:
	class Radian;
	class Degree : public Carrier{
	public:
		Degree();
		Degree(const double val);
		explicit operator double() const;
		explicit operator Radian() const;
	};
	class Radian : public Carrier{
	public:
		Radian();
		Radian(const double val);
		explicit operator double() const;
		explicit operator Degree() const;
	};
public:
	enum struct E_Left_State{E_Open, E_Close, E_Angle};
	enum struct E_Right_State{E_Open, E_Close, E_Angle};

private:
	static constexpr E_Left_State INIT_LEFT_STATE = E_Left_State::E_Open;
	static constexpr E_Right_State INIT_RIGHT_STATE = E_Right_State::E_Open;
	static constexpr int OUTPUT_NUMBER_OF_DECIMAL_PLACES = 2;

	static const std::string MSG_LEFT_OPEN;
	static const std::string MSG_RIGHT_OPEN;
	static const std::string MSG_BOTH_OPEN;
	static const std::string MSG_BOTH_CLOSE;
	static const std::string MSG_LEFT_ANGLE;
	static const std::string MSG_RIGHT_ANGLE;

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
	 * @brief Reverce left state (Open <-> Close).
	 */
	void left_reverce();
	/**
	 * @brief Reverce right state (Open <-> Close).
	 */
	void right_reverce();
	/**
	 * @brief setter.
	 * @param state Gripper state (E_Open or E_Close).
	 */
	void set_state(const E_Left_State & state);
	/**
	 * @brief setter.
	 * @param state Gripper state (E_Open or E_Close).
	 */
	void set_state(const E_Right_State & state);
	/**
	 * @brief setter. Gripper will open designated angle
	 * @param state Specify E_Angle.
	 * @param angle [degree].
	 */
	void set_state(const E_Left_State & state, const Degree& angle);
	/**
	 * @brief setter Gripper will open designated angle.
	 * @param state Specify E_Angle
	 * @param angle [degree].
	 */
	void set_state(const E_Right_State & state, const Degree& angle);
	/**
	 * @brief setter. Gripper will open designated angle.
	 * @param state Specify E_Angle
	 * @param angle [radian].
	 */
	void set_state(const E_Left_State & state, const Radian& angle);
	/**
	 * @brief setter. Gripper will open designated angle.
	 * @param state Specify E_Angle.
	 * @param angle [radian].
	 */
	void set_state(const E_Right_State & state, const Radian& angle);
	/**
	 * @brief setter. 
	 * @param left_state Gripper state (E_Open or E_Close)
	 * @param right_state Gripper state (E_Open or E_Close)
	 */
	void set_state(const E_Left_State & left_state, const E_Right_State & right_state);
	/**
	 * @brief setter
	 * @param left_angle [degree]
	 * @param right_angle [degree]
	 */
	void set_state(const Degree & left_angle, const Degree & right_angle);
	/**
	 * @brief setter
	 * @param left_angle [radian]
	 * @param right_angle [radian]
	 */
	void set_state(const Radian & left_angle, const Radian & right_angle);

private:
	void publish();
	void publish(const E_Left_State& state, const double angle);
	void publish(const E_Right_State& state, const double angle);
};

#endif