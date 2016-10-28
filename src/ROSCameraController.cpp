/**
 * @file ROSCameraController.cpp
 * @brief 
 *	Implimentation of ROSCameraController
 * @author Iwase
 * @date 2016.10.25.
 */
//----------------------------------------------------------
// Include
//----------------------------------------------------------
#include "ROSCameraController.hpp"

constexpr char ROSCameraController::CameraController::service_open_name[];
constexpr char ROSCameraController::CameraController::service_close_name[];

constexpr char ROSCameraController::HeadCameraHandler::component_id[];
constexpr char ROSCameraController::LeftHandCameraHandler::component_id[];
constexpr char ROSCameraController::RightHandCameraHandler::component_id[];
constexpr char ROSCameraController::HeadCameraHandler::topic_name[];
constexpr char ROSCameraController::LeftHandCameraHandler::topic_name[];
constexpr char ROSCameraController::RightHandCameraHandler::topic_name[];
constexpr char ROSCameraController::service_cameras_list_name[];

void ROSCameraController::CameraController::init()
{
	assert(!initialized_);
	camera_opener_.init();
	camera_closer_.init();
	initialized_ = true;
};
bool ROSCameraController::CameraController::open_camera()
{
	assert(initialized_);
	ROS_INFO("CameraController::open_camera - open_camera");
	baxter_core_msgs::OpenCamera msg;
	msg.request.name = component_id_;
	if (!camera_opener_.call(msg)){
  		ROS_ERROR("CameraController::open_camera - Failed to call service");
  		return false;
  	}
	if(msg.response.err != 0) 
		ROS_ERROR("CameraController::open_camera - FAILED to open camera : %s , %d", component_id_.c_str(), msg.response.err);
	return msg.response.err != 0 ? true : false;
};
bool ROSCameraController::CameraController::close_camera()
{
	assert(initialized_);
	ROS_INFO("CameraController::close_camera - close_camera");
	baxter_core_msgs::CloseCamera msg;
	msg.request.name = component_id_;
	if (!camera_closer_.call(msg)){
  		ROS_ERROR("CameraController::close_camera - Failed to call service");
  		return false;
  	}
	if(msg.response.err != 0) 
		ROS_ERROR("CameraController::close_camera - FAILED to close camera : %s , %d", component_id_.c_str(), msg.response.err);
	return msg.response.err != 0 ? true : false;
};
ROSCameraController::ROSCameraController() : 
	initialized_(false),
	cameras_list_service_(service_cameras_list_name)
{
	head_handler.active = false;
	left_handler.active = true;
	right_handler.active = true;
};
ROSCameraController::ROSCameraController(const E_Camera cam) :
	initialized_(false),
	cameras_list_service_(service_cameras_list_name)
{
	head_handler.active = false;
	left_handler.active = false;
	right_handler.active = false;
	(get_handler_pointer(cam))->active = true;
};
ROSCameraController::ROSCameraController(const E_Camera cam1, const E_Camera cam2) : 
	initialized_(false),
	cameras_list_service_(service_cameras_list_name)
{
	assert(cam1 != cam2);
	head_handler.active = false;
	left_handler.active = false;
	right_handler.active = false;
	(get_handler_pointer(cam1))->active = true;
	(get_handler_pointer(cam2))->active = true;
};
void ROSCameraController::init()
{
	assert(!initialized_);
	cameras_list_service_.init();
	// Instanciate Node handle 
	head_handler.controller.init();
	left_handler.controller.init();
	right_handler.controller.init();

	// Default left_hand_camera, right_hand_camera is active
	{
		std::list<E_Camera> devicies;
		if(!get_active_device(devicies)){
	  		ROS_ERROR("ROSCameraController::init - Failed to get devicies");
			exit(EXIT_FAILURE);
		}
		for(auto dev : devicies){
			if(!(get_handler_pointer(dev))->active){
				if(!(get_handler_pointer(dev))->controller.close_camera())
					(get_handler_pointer(dev))->active = true;
			}
		}	
	}
	{
		std::list<E_Camera> devicies;
		if(!get_not_active_device(devicies)){
	  		ROS_ERROR("ROSCameraController::init - Failed to get devicies");
			exit(EXIT_FAILURE);
		}
		for(auto dev : devicies){
			if((get_handler_pointer(dev))->active){
				if(!(get_handler_pointer(dev))->controller.open_camera())
					(get_handler_pointer(dev))->active = false;
			}
		}	
	}

	// start the image subscriber from active camera
	if(head_handler.active){
		head_handler.image_getter_p.reset( new ROSSubscriberInterface<sensor_msgs::Image>(HeadCameraHandler::topic_name) );
		head_handler.image_getter_p->init();
	}
	if(left_handler.active){
		left_handler.image_getter_p.reset( new ROSSubscriberInterface<sensor_msgs::Image>(LeftHandCameraHandler::topic_name) );
		left_handler.image_getter_p->init();
	}
	if(right_handler.active){
		right_handler.image_getter_p.reset( new ROSSubscriberInterface<sensor_msgs::Image>(RightHandCameraHandler::topic_name) );
		right_handler.image_getter_p->init();
	}

	initialized_ = true;
};
void ROSCameraController::open_camera(const E_Camera camera)
{
	assert(initialized_);
	// assert(get_active_num() < ROSCameraController::MAX_CAMERA_NUM);
	std::cout << get_active_num() << std::endl;
	if((get_active_num() >= ROSCameraController::MAX_CAMERA_NUM)) {
		ROS_WARN("ROSCameraController::open_camera - Open failed. Two cameras can be activated at same time.");
	}
	else{
		switch(camera){
			case ROSCameraController::E_Camera::E_Head:
				if(!head_handler.active){
					head_handler.active = true;
					head_handler.controller.open_camera();				
					head_handler.image_getter_p.reset( new ROSSubscriberInterface<sensor_msgs::Image>(HeadCameraHandler::topic_name) );
					head_handler.image_getter_p->init();
				}
				break;
			case ROSCameraController::E_Camera::E_LeftHand:
				if(!left_handler.active){
					left_handler.active = true;
					left_handler.controller.open_camera();
					left_handler.image_getter_p.reset( new ROSSubscriberInterface<sensor_msgs::Image>(LeftHandCameraHandler::topic_name) );
					left_handler.image_getter_p->init();
				}
				break;
			case ROSCameraController::E_Camera::E_RightHand:
				if(!right_handler.active){
					right_handler.active = true;
					right_handler.controller.open_camera();
					right_handler.image_getter_p.reset( new ROSSubscriberInterface<sensor_msgs::Image>(RightHandCameraHandler::topic_name) );
					right_handler.image_getter_p->init();
				}
				break;
		}
	}
};
void ROSCameraController::close_camera(const E_Camera camera)
{
	assert(initialized_);
	switch(camera){
		case ROSCameraController::E_Camera::E_Head:
			if(head_handler.active){
				head_handler.active = false;
				head_handler.controller.close_camera();
				head_handler.image_getter_p.reset();
			}
			break;
		case ROSCameraController::E_Camera::E_LeftHand:
			if(left_handler.active){
				left_handler.active = false;
				left_handler.controller.close_camera();
				left_handler.image_getter_p.reset();
			}
			break;
		case ROSCameraController::E_Camera::E_RightHand:
			if(right_handler.active){
				right_handler.active = false;
				right_handler.controller.close_camera();
				right_handler.image_getter_p.reset();
			}
			break;
		default:
			break;
	}
};
bool ROSCameraController::is_active(const E_Camera camera)
{
	return (get_handler_pointer(camera))->active;
};
bool ROSCameraController::new_data(const E_Camera camera)
{	
	bool new_data_available = false;
	
	if( is_active(camera) && (get_handler_pointer(camera))->image_getter_p )
		new_data_available = (get_handler_pointer(camera))->image_getter_p->new_data();
	
	return new_data_available;
};
void ROSCameraController::get_image(sensor_msgs::Image& image, const E_Camera camera)
{
	assert(initialized_);
	assert(is_active(camera) && (get_handler_pointer(camera))->image_getter_p );
	
	(get_handler_pointer(camera))->image_getter_p->get(image);
};
void ROSCameraController::get_image(cv_bridge::CvImagePtr& image, const E_Camera camera, const std::string& encoding)
{
	assert(initialized_);
	assert(is_active(camera) && (get_handler_pointer(camera))->image_getter_p );
	
	// sensor_msgs to cvImagePtr (make copy)
	sensor_msgs::Image msg;
	(get_handler_pointer(camera))->image_getter_p->get(msg);
	try{
		image = cv_bridge::toCvCopy(msg, encoding);
	}
	catch (cv_bridge::Exception& e){
		ROS_ERROR("ROSCameraController::get_image - cv_bridge exception: %s", e.what());
		return;
	}
};
void ROSCameraController::get_image(cv::Mat& image, const E_Camera camera, const std::string& encoding)
{
	assert(initialized_);
	assert(is_active(camera) && (get_handler_pointer(camera))->image_getter_p );

	sensor_msgs::Image msg;
	(get_handler_pointer(camera))->image_getter_p->get(msg);

	// sensor_msgs to cvImagePtr (make copy)
	cv_bridge::CvImagePtr bridge;
	try{
		bridge = cv_bridge::toCvCopy(msg, encoding);
	}
	catch (cv_bridge::Exception& e){
		ROS_ERROR("ROSCameraController::get_image - cv_bridge exception: %s", e.what());
		return;
	}

	// cvImagePtr to cv::Mat (make copy)
	bridge->image.copyTo(image);
};

bool ROSCameraController::get_active_device(std::list<E_Camera>& devicies)
{
	// get available cameras
	baxter_core_msgs::ListCameras msg;
	if (!cameras_list_service_.call(msg)){
	 	ROS_ERROR("ROSCameraController::get_active_device - Failed to call service");
	  	return false;
	}
	
	// get all topic
	ros::master::V_TopicInfo topics;
	ros::master::getTopics(topics);

	// search camera topic
	for(auto topic : topics){
		if(topic.name == std::string(LeftHandCameraHandler::topic_name))
			devicies.push_back(ROSCameraController::E_Camera::E_LeftHand);
		else if(topic.name == std::string(RightHandCameraHandler::topic_name))
			devicies.push_back(ROSCameraController::E_Camera::E_RightHand);
		else if(topic.name == std::string(HeadCameraHandler::topic_name))
			devicies.push_back(ROSCameraController::E_Camera::E_Head);
	}
	return true;
};
bool ROSCameraController::get_not_active_device(std::list<E_Camera>& devicies)
{
	std::list<E_Camera> active_device;
	get_active_device(active_device);
	
	devicies.push_back(ROSCameraController::E_Camera::E_LeftHand);
	devicies.push_back(ROSCameraController::E_Camera::E_RightHand);
	devicies.push_back(ROSCameraController::E_Camera::E_Head);

	for(auto a : active_device){
		devicies.remove(a);
	}
	return true;
};

unsigned int ROSCameraController::get_active_num()
{
	std::list<E_Camera> active_device;
	get_active_device(active_device);
	return active_device.size();
};
ROSCameraController::CameraHandler* ROSCameraController::get_handler_pointer(const E_Camera camera){
	ROSCameraController::CameraHandler* ref;
	switch(camera){
		case ROSCameraController::E_Camera::E_Head:
			ref = &head_handler;
			break;
		case ROSCameraController::E_Camera::E_LeftHand:
			ref = &left_handler;
			break;
		case ROSCameraController::E_Camera::E_RightHand:
			ref = &right_handler;
			break;
		default:
			break;
	}
	return ref;
};