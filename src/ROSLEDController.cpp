/**
 * @file ROSLEDController.cpp
 * @brief 
 *	Implimentation of ROSLEDController
 * @author Iwase
 */
//----------------------------------------------------------
// Include
//----------------------------------------------------------
#include <ros_module/ROSInterface.hpp>
//----------------------------------------------------------
// Constant parameter
//----------------------------------------------------------
const unsigned int ROSLEDController::PUBLISH_FREQUENCY = 120;	// Controll loop runs on this frequency
const unsigned int ROSLEDController::DEFAULT_BEHAVIOR = 0x0000;
const unsigned int ROSLEDController::OVERRIDE_ENABLE  = 0x8000;
//----------------------------------------------------------
// ROSLEDController
//----------------------------------------------------------
ROSLEDController::ROSLEDController() : 
	thread_exit(false),
	control_enable(true)
{};
ROSLEDController::~ROSLEDController()
{
	mtx.lock();
	thread_exit = true;
	mtx.unlock();
	if(controll_thread.joinable()) controll_thread.join();
};
void ROSLEDController::init()
{
	ros::NodeHandle n;
	// L,R command publisher
	publisher = n.advertise<SMUint16>("robot/sonar/head_sonar/lights/set_lights", 120);

	// Start publish loop
	try {
		controll_thread = std::thread( [this]{ publish_loop(); } );
	}
	catch (std::system_error& e) {
		std::cout << e.what() << std::endl;
	}
};
void ROSLEDController::enable()
{
	std::lock_guard<std::mutex> lock(mtx);
	control_enable = true;
};
void ROSLEDController::disable()
{	
	std::lock_guard<std::mutex> lock(mtx);
	control_enable = false;
};
void ROSLEDController::set_command(const std::array<bool, LED_NUM>& c)
{
	std::lock_guard<std::mutex> lock(mtx);
	std::copy_n(c.begin(), LED_NUM, state.begin());
};
void ROSLEDController::publish_loop()
{
	SMUint16 message;
	ros::Rate loop_timer(PUBLISH_FREQUENCY);
	// publish loop
	bool loop_continue(true);
	while(loop_continue){
		mtx.lock();
		loop_continue = !thread_exit;
		message = to_message(state);
		mtx.unlock();

		publisher.publish(message);
		loop_timer.sleep(); //sleep
	}
};
SMUint16 ROSLEDController::to_message(const std::array<bool, LED_NUM>& a) const
{
	SMUint16 msg;
	msg.data = 0x0000;
	if(control_enable)
		msg.data |= OVERRIDE_ENABLE;
	for(int i = 0; i < LED_NUM; ++ i){
		if(state[i])
			msg.data |= (0x0001 << i);
	}
	return msg;
};