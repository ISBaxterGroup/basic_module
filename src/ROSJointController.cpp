//----------------------------------------------------------
// Include
//----------------------------------------------------------
#include "ROSJointController.hpp"
//----------------------------------------------------------
// Constant parameter
//----------------------------------------------------------
const unsigned int ROSJointController::POSITION_MODE = 1; 	// Joint controll mode  (position mode)
const unsigned int ROSJointController::PUBLISH_FREQUENCY = 100;	// Controll loop runs on this frequency
const double ROSJointController::SPEED_RATIO = 0.15;	// The move speed of motor  0.0 - 1.0 (default 0.3)  
const double ROSJointController::CMD_TIMEOUT = 1.0;	// Time out until controll mode will losts

// Names in arm joints
const std::array<std::string, ROSJointController::JOINTS_NUM> ROSJointController::LEFT_JOINT_NAMES = {"left_s0", "left_s1", "left_e0", "left_e1", "left_w0", "left_w1", "left_w2"};
const std::array<std::string, ROSJointController::JOINTS_NUM> ROSJointController::RIGHT_JOINT_NAMES = {"right_s0", "right_s1", "right_e0", "right_e1", "right_w0", "right_w1", "right_w2"};

// Initial angles in arm joints
const std::array<double, ROSJointController::JOINTS_NUM> ROSJointController::INIT_LEFT_JOINT_COMMAND = { 0.362528, -1.24796, -0.894286, 1.52721, 0.211943, 1.35341, 1.73632 };
const std::array<double, ROSJointController::JOINTS_NUM> ROSJointController::INIT_RIGHT_JOINT_COMMAND = { -0.783666, -0.840139, 1.54499, 1.59949, -0.71994, 1.50835, 1.24368 };


//----------------------------------------------------------
// LRJCommand
//----------------------------------------------------------
ROSJointController::LRJCommand::LRJCommand(const BCMJointCommand& jc)
{
	command.resize(ROSJointController::JOINTS_NUM);
	copy_n(jc.command.begin(), ROSJointController::JOINTS_NUM, this->command.begin());
};
ROSJointController::LRJCommand::LRJCommand(const std::array<double, ROSJointController::JOINTS_NUM>& a)
{
	command.resize(ROSJointController::JOINTS_NUM);
	copy_n(a.begin(), ROSJointController::JOINTS_NUM, this->command.begin());
};

ROSJointController::LeftJCommand::LeftJCommand(const BCMJointCommand& jc) : LRJCommand(jc){};
ROSJointController::LeftJCommand::LeftJCommand(const std::array<double, ROSJointController::JOINTS_NUM> & a) : LRJCommand(a){};
ROSJointController::RightJCommand::RightJCommand(const BCMJointCommand& jc) : LRJCommand(jc){};
ROSJointController::RightJCommand::RightJCommand(const std::array<double, ROSJointController::JOINTS_NUM> & a) : LRJCommand(a){};

//----------------------------------------------------------
// ROSJointController
//----------------------------------------------------------
ROSJointController::ROSJointController():
	exit(false)
{
	// Set initial value
	left_joint_cmd.names.resize(JOINTS_NUM);
	right_joint_cmd.names.resize(JOINTS_NUM);
	left_joint_cmd.command.resize(JOINTS_NUM);
	right_joint_cmd.command.resize(JOINTS_NUM);
};
ROSJointController::~ROSJointController()
{
	mtx.lock();
	exit = true;
	mtx.unlock();
	if(controll_thread.joinable()) controll_thread.join();
};
void ROSJointController::init()
{
	ros::NodeHandle n;
	// Publish rate publisher
	pub_rate = n.advertise<SMUInt16>("robot/joint_state_publish_rate", 1);

	// Speed ratio publisher
	pub_left_speed_ratio = n.advertise<SMFloat64>("robot/limb/left/set_speed_ratio", 1);
	pub_right_speed_ratio = n.advertise<SMFloat64>("robot/limb/right/set_speed_ratio", 1);

	// Timeout publisher
	pub_left_cmd_timeout = n.advertise<SMFloat64>("robot/limb/left/joint_command_timeout", 1);
	pub_right_cmd_timeout = n.advertise<SMFloat64>("robot/limb/right/joint_command_timeout", 1);

	// L,R command publisher
	pub_left_cmd = n.advertise<BCMJointCommand>("robot/limb/left/joint_command", 32);
	pub_right_cmd = n.advertise<BCMJointCommand>("robot/limb/right/joint_command", 32);

	// Start publish loop
	try {
		controll_thread = std::thread( [this]{ publish_loop(); } );
	}
	catch (std::system_error& e) {
		std::cout << e.what() << std::endl;
	}

}
void ROSJointController::set_command(const LeftJCommand& c)
{
	std::lock_guard<std::mutex> lock(mtx);
	copy_n(c.command.begin(), JOINTS_NUM, left_joint_cmd.command.begin());
};
void ROSJointController::set_command(const RightJCommand& c)
{
	std::lock_guard<std::mutex> lock(mtx);
	copy_n(c.command.begin(), JOINTS_NUM, right_joint_cmd.command.begin());
};
void ROSJointController::publish_loop()
{
	ros::Rate loop_timer(PUBLISH_FREQUENCY);

	left_joint_cmd.mode = POSITION_MODE;
	right_joint_cmd.mode = POSITION_MODE;
	
	copy_n(LEFT_JOINT_NAMES.begin(), JOINTS_NUM, left_joint_cmd.names.begin());
	copy_n(RIGHT_JOINT_NAMES.begin(), JOINTS_NUM, right_joint_cmd.names.begin());

	copy_n(INIT_LEFT_JOINT_COMMAND.begin(), JOINTS_NUM, left_joint_cmd.command.begin());
	copy_n(INIT_RIGHT_JOINT_COMMAND.begin(), JOINTS_NUM, right_joint_cmd.command.begin());

	// Set the move speed
	SMFloat64 speed_ratio;
	speed_ratio.data = SPEED_RATIO;

	// Set the update frequency
	SMUInt16 freq;
	freq.data = PUBLISH_FREQUENCY;
	pub_rate.publish(freq);

	// Set the time out until controll mode in baxter will losts
	SMFloat64 timeout;
	timeout.data = CMD_TIMEOUT;
	pub_left_cmd_timeout.publish(timeout);
	pub_right_cmd_timeout.publish(timeout);

	// publish loop
	while(1){
		mtx.lock();

		pub_left_speed_ratio.publish(speed_ratio);
		pub_right_speed_ratio.publish(speed_ratio);

		pub_left_cmd.publish(left_joint_cmd);
   		pub_right_cmd.publish(right_joint_cmd);

		if(exit) break;

		mtx.unlock();
		loop_timer.sleep(); //sleep
	}
};

