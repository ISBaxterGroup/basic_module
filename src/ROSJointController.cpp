/**
 * @file ROSJointController.cpp
 * @brief 
 *	Implimentation of ROSJointController
 * @author Iwase
 */
//----------------------------------------------------------
// Include
//----------------------------------------------------------
#include "ROSJointController.hpp"
//----------------------------------------------------------
// Constant parameter
//----------------------------------------------------------
const unsigned int ROSJointController::POSITION_MODE = 1; 	// Joint controll mode  (position mode)
const unsigned int ROSJointController::PUBLISH_FREQUENCY = 100;	// Controll loop runs on this frequency (must be over 5)
const double ROSJointController::SPEED_RATIO = 0.15;	// The move speed of motor  0.0 - 1.0 (default 0.3)  
const double ROSJointController::CMD_TIMEOUT = 1.0;	// Time out until controll mode will losts

// Names in arm joints
const std::array<std::string, ROSJointController::JOINTS_NUM> ROSJointController::LEFT_JOINT_NAMES = {"left_s0", "left_s1", "left_e0", "left_e1", "left_w0", "left_w1", "left_w2"};
const std::array<std::string, ROSJointController::JOINTS_NUM> ROSJointController::RIGHT_JOINT_NAMES = {"right_s0", "right_s1", "right_e0", "right_e1", "right_w0", "right_w1", "right_w2"};
const std::array<std::string, ROSJointController::OTHER_JOINTS_NUM> OTHER_JOINT_NAMES  = {"head_nod", "head_pan", "torso_t0"};
// Initial angles in arm joints
const std::array<double, ROSJointController::JOINTS_NUM> ROSJointController::INIT_LEFT_JOINT_COMMAND = { 0.362528, -1.24796, -0.894286, 1.52721, 0.211943, 1.35341, 1.73632 };
const std::array<double, ROSJointController::JOINTS_NUM> ROSJointController::INIT_RIGHT_JOINT_COMMAND = { -0.783666, -0.840139, 1.54499, 1.59949, -0.71994, 1.50835, 1.24368 };

//----------------------------------------------------------
// LRJCommand
//----------------------------------------------------------
ROSJointController::LRJCommand::LRJCommand(const baxter_core_msgs::JointCommand& jc)
{
	command.resize(ROSJointController::JOINTS_NUM);
	copy_n(jc.command.begin(), ROSJointController::JOINTS_NUM, this->command.begin());
};
ROSJointController::LRJCommand::LRJCommand(const std::array<double, ROSJointController::JOINTS_NUM>& a)
{
	command.resize(ROSJointController::JOINTS_NUM);
	copy_n(a.begin(), ROSJointController::JOINTS_NUM, this->command.begin());
};

ROSJointController::LeftJCommand::LeftJCommand(const baxter_core_msgs::JointCommand& jc) : LRJCommand(jc){};
ROSJointController::LeftJCommand::LeftJCommand(const std::array<double, ROSJointController::JOINTS_NUM> & a) : LRJCommand(a){};
ROSJointController::RightJCommand::RightJCommand(const baxter_core_msgs::JointCommand& jc) : LRJCommand(jc){};
ROSJointController::RightJCommand::RightJCommand(const std::array<double, ROSJointController::JOINTS_NUM> & a) : LRJCommand(a){};

//----------------------------------------------------------
// ROSJointController
//----------------------------------------------------------
ROSJointController::ROSJointController():
	exit_pub_(false),
	exit_sub_(false),
	enable_collision_avoidance(true)
{
	// Set initial value
	left_joint_cmd.names.resize(JOINTS_NUM);
	right_joint_cmd.names.resize(JOINTS_NUM);
	left_joint_cmd.command.resize(JOINTS_NUM);
	right_joint_cmd.command.resize(JOINTS_NUM);

	// Buffer for recieve
	curr_joint_state.name.resize(JOINTS_NUM * 2 + OTHER_JOINTS_NUM);
	curr_joint_state.position.resize(JOINTS_NUM * 2 + OTHER_JOINTS_NUM);
	curr_joint_state.velocity.resize(JOINTS_NUM * 2 + OTHER_JOINTS_NUM);
	curr_joint_state.effort.resize(JOINTS_NUM * 2 + OTHER_JOINTS_NUM);
};
ROSJointController::~ROSJointController()
{
	// exit publish loop
	mtx_pub.lock();
	exit_pub_ = true;
	mtx_pub.unlock();

	// exit subscribe loop
	mtx_sub.lock();
	exit_sub_ = true;
	mtx_sub.unlock();

	// joint thread
	if(controll_thread.joinable()) controll_thread.join();
	if(subscribe_thread.joinable()) subscribe_thread.join();
};
void ROSJointController::init()
{
	ros::NodeHandle n;
	// Publish rate publisher
	pub_rate = n.advertise<std_msgs::UInt16>("robot/joint_state_publish_rate", 1);

	// Speed ratio publisher
	pub_left_speed_ratio = n.advertise<std_msgs::Float64>("robot/limb/left/set_speed_ratio", 1);
	pub_right_speed_ratio = n.advertise<std_msgs::Float64>("robot/limb/right/set_speed_ratio", 1);

	// Timeout publisher
	pub_left_cmd_timeout = n.advertise<std_msgs::Float64>("robot/limb/left/joint_command_timeout", 1);
	pub_right_cmd_timeout = n.advertise<std_msgs::Float64>("robot/limb/right/joint_command_timeout", 1);

	// L,R command publisher
	pub_left_cmd = n.advertise<baxter_core_msgs::JointCommand>("robot/limb/left/joint_command", 32);
	pub_right_cmd = n.advertise<baxter_core_msgs::JointCommand>("robot/limb/right/joint_command", 32);

	// stop collision avoidance publisher
	pub_left_stop_collision_avoidance = n.advertise<std_msgs::Empty>("/robot/limb/left/suppress_collision_avoidance", 1);
	pub_right_stop_collision_avoidance = n.advertise<std_msgs::Empty>("/robot/limb/right/suppress_collision_avoidance", 1);

	// joint state subscriber
	sub_joint_state = n.subscribe("/robot/joint_states", 32, &ROSJointController::sub_state_callback, this);

	// Start publish loop
	try {
		controll_thread = std::thread( [this]{ publish_loop(); } );
	}
	catch (std::system_error& e) {
		std::cout << e.what() << std::endl;
	}
	// Start publish loop
	try {
		subscribe_thread = std::thread( [this]{ subscribe_loop(); } );
	}
	catch (std::system_error& e) {
		std::cout << e.what() << std::endl;
	}
}
void ROSJointController::collision_avoidance(const bool enable)
{
	std::lock_guard<std::mutex> lock(mtx_pub);
	enable_collision_avoidance = enable; 
};
void ROSJointController::set_command(const LeftJCommand& c)
{
	std::lock_guard<std::mutex> lock(mtx_pub);
	copy_n(c.command.begin(), JOINTS_NUM, left_joint_cmd.command.begin());
};
void ROSJointController::set_command(const RightJCommand& c)
{
	std::lock_guard<std::mutex> lock(mtx_pub);
	copy_n(c.command.begin(), JOINTS_NUM, right_joint_cmd.command.begin());
};
void ROSJointController::get_left_joint_angle(std::array<double, JOINTS_NUM> & left_joint_angle)
{
	std::lock_guard<std::mutex> lock(mtx_sub);
	// sort by name
	for(unsigned int i = 0; i < JOINTS_NUM; ++ i){
		for(unsigned int j = 0; j < JOINTS_NUM * 2 + OTHER_JOINTS_NUM; ++ j){
			if(curr_joint_state.name[j] == LEFT_JOINT_NAMES[i]){
				left_joint_angle[i] = curr_joint_state.position[j];
				break;
			}
		}
	}

};
void ROSJointController::get_right_joint_angle(std::array<double, JOINTS_NUM> & right_joint_angle)
{
	std::lock_guard<std::mutex> lock(mtx_sub);
	// sort by name
	for(unsigned int i = 0; i < JOINTS_NUM; ++ i){
		for(unsigned int j = 0; j < JOINTS_NUM * 2 + OTHER_JOINTS_NUM; ++ j){
			if(curr_joint_state.name[j] == RIGHT_JOINT_NAMES[i]){
				right_joint_angle[i] = curr_joint_state.position[j];
				break;
			}
		}
	}
};
void ROSJointController::get_joint_angle(std::array<double, JOINTS_NUM> & left_joint_angle, std::array<double, JOINTS_NUM> & right_joint_angle)
{
	get_left_joint_angle(left_joint_angle);
	get_right_joint_angle(right_joint_angle);
};
void ROSJointController::sub_state_callback(const sensor_msgs::JointState& joint_state)
{
	std::lock_guard<std::mutex> lock(mtx_sub);

	std::copy_n(joint_state.name.begin(), JOINTS_NUM * 2 + OTHER_JOINTS_NUM, curr_joint_state.name.begin());
	std::copy_n(joint_state.position.begin(), JOINTS_NUM * 2 + OTHER_JOINTS_NUM, curr_joint_state.position.begin());
	std::copy_n(joint_state.velocity.begin(), JOINTS_NUM * 2 + OTHER_JOINTS_NUM, curr_joint_state.velocity.begin());
	std::copy_n(joint_state.effort.begin(), JOINTS_NUM * 2 + OTHER_JOINTS_NUM, curr_joint_state.effort.begin());
}
void ROSJointController::publish_loop()
{
	left_joint_cmd.mode = POSITION_MODE;
	right_joint_cmd.mode = POSITION_MODE;
	
	copy_n(LEFT_JOINT_NAMES.begin(), JOINTS_NUM, left_joint_cmd.names.begin());
	copy_n(RIGHT_JOINT_NAMES.begin(), JOINTS_NUM, right_joint_cmd.names.begin());

	copy_n(INIT_LEFT_JOINT_COMMAND.begin(), JOINTS_NUM, left_joint_cmd.command.begin());
	copy_n(INIT_RIGHT_JOINT_COMMAND.begin(), JOINTS_NUM, right_joint_cmd.command.begin());

	// Set the move speed
	std_msgs::Float64 speed_ratio;
	speed_ratio.data = SPEED_RATIO;

	// Set the update frequency
	std_msgs::UInt16 freq;
	freq.data = PUBLISH_FREQUENCY;
	pub_rate.publish(freq);

	// Set the time out until controll mode in baxter will losts
	std_msgs::Float64 timeout;
	timeout.data = CMD_TIMEOUT;
	pub_left_cmd_timeout.publish(timeout);
	pub_right_cmd_timeout.publish(timeout);

	ros::Rate loop_timer(PUBLISH_FREQUENCY);

	bool loop_continue(true);
	// publish loop
	while(loop_continue){
		mtx_pub.lock();
		
		if(!enable_collision_avoidance){
			pub_left_stop_collision_avoidance.publish(std_msgs::Empty());
			pub_right_stop_collision_avoidance.publish(std_msgs::Empty());
		}	

		pub_left_speed_ratio.publish(speed_ratio);
		pub_right_speed_ratio.publish(speed_ratio);

		pub_left_cmd.publish(left_joint_cmd);
   		pub_right_cmd.publish(right_joint_cmd);

		loop_continue = !exit_pub_;

		mtx_pub.unlock();
		loop_timer.sleep(); //sleep
	}
};
void ROSJointController::subscribe_loop()
{
	bool loop_continue(true);

	// subscribe loop
	while(loop_continue){
		// If Call Available run call back method.
		ros::spinOnce();

		mtx_sub.lock();
		loop_continue = !exit_sub_;
		mtx_sub.unlock();

	}
};