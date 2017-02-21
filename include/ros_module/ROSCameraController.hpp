/**
 * @file ROSCameraController.hpp
 * @brief 
 *	This class provides easy access to three cameras placed on baxter.
 *	(Right-hand, Left-hand, Head)
 * @author Iwase
 * @date 2016.9.25.
 */
//----------------------------------------------------------
// Camera controller
//----------------------------------------------------------
#ifndef ROS_CAMERA_CONTROLLER_HPP
#define ROS_CAMERA_CONTROLLER_HPP

//----------------------------------------------------------
// include
//----------------------------------------------------------
#include <iostream>
#include <memory>
#include <string>
#include <mutex>
#include <thread>
#include <vector>
#include <list>

#include <ros/ros.h>
#include <ros/master.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <baxter_core_msgs/ListCameras.h>
#include <baxter_core_msgs/CloseCamera.h>
#include <baxter_core_msgs/OpenCamera.h>

#include "ROSFramework.hpp"
/**
* @class ROSCameraController
* @brief An interface with cameras on baxter.
*		 WARNING: Two or more instantiated is not assumed.
*/
class ROSCameraController{
private:
	struct CameraController{
		private:
			static constexpr char service_open_name[] = "/cameras/open";
			static constexpr char service_close_name[] = "/cameras/close";

		private:
			const std::string component_id_;

		private:
			bool initialized_;
			ROSServiceInterface<baxter_core_msgs::OpenCamera> camera_opener_;
			ROSServiceInterface<baxter_core_msgs::CloseCamera> camera_closer_;

		public:
			CameraController(const std::string& component_id):
				initialized_(false),
				component_id_(component_id),
				camera_opener_(ROSCameraController::CameraController::service_open_name),
				camera_closer_(ROSCameraController::CameraController::service_close_name)
			{};

			void init();
			bool open_camera();
			bool close_camera();
	};

	struct CameraHandler{
		bool active;
		CameraController controller;
		std::unique_ptr< ROSSubscriberInterface<sensor_msgs::Image> > image_getter_p;
		CameraHandler(const bool init_state, const std::string& id) : active(init_state), controller(id), image_getter_p(nullptr) {};
	};

	struct HeadCameraHandler : CameraHandler{
		static constexpr bool init_state = false;
		static constexpr char component_id[] = "head_camera";
		static constexpr char topic_name[] = "/cameras/head_camera/image";

		HeadCameraHandler() : CameraHandler(init_state, component_id) {};
	};

	struct LeftHandCameraHandler : CameraHandler{
		static constexpr bool init_state = false;
		static constexpr char component_id[] = "left_hand_camera";
		static constexpr char topic_name[] = "/cameras/left_hand_camera/image";

		LeftHandCameraHandler() : CameraHandler(init_state, component_id) {};
	};
	struct RightHandCameraHandler : CameraHandler{
		static constexpr bool init_state = false;
		static constexpr char component_id[] = "right_hand_camera";
		static constexpr char topic_name[] = "/cameras/right_hand_camera/image";

		RightHandCameraHandler() : CameraHandler(init_state, component_id) {};
	};

public:
	/**
	 * @enum E_Camera
	 * @brief camera position.
	 */
	enum struct E_Camera{
		E_Head,
		E_LeftHand,
		E_RightHand
	};

public:
	// ! The number of cameras that can be enabled at the same time
	static constexpr int MAX_CAMERA_NUM = 2;

private:
	static constexpr char service_cameras_list_name[] = "/cameras/list";
	bool initialized_;
	
	ROSServiceInterface<baxter_core_msgs::ListCameras> cameras_list_service_;

	HeadCameraHandler head_handler;
	LeftHandCameraHandler left_handler;
	RightHandCameraHandler right_handler;

public:
	/**
	 * @brief A constructor.
	 */
	ROSCameraController();
	/**
	 * @brief A constructor from active camera.
	 * @param cam camera to use.
	 */
	ROSCameraController(const E_Camera cam);
	/**
	 * @brief A constructor from two active camera.
	 * @param cam1 camera to use.
	 * @param cam1 camera to use.
	 */
	ROSCameraController(const E_Camera cam1, const E_Camera cam2);
	/**
	 * @brief Initialize node and Start the subscribe thread
	 */
	void init();
	/**
	 * @brief open camera. WARNING: Max Two camera can be opened at same time.
	 * @param camera target camera
	 */
	void open_camera(const E_Camera camera);
	/**
	 * @brief close camera.
	 * @param camera target camera
	 */
	void close_camera(const E_Camera camera);
	/**
	 * @brief Returns whether or camera is valid
	 * @param camera target camera
	 * @return active : true, not active : false
	 */
	bool is_active(const E_Camera camera);
	/**
	 * @brief Returns whether or new data came.
	 * @param camera target camera
	 * @return new data came : true, not updated : false
	 */
	bool new_data(const E_Camera camera);
	/**
	 * @brief getter
	 * @param image return value
	 * @param camera target camera
	 */
	void get_image(sensor_msgs::Image& image, const E_Camera camera);
	/**
	 * @brief getter
	 * @param image return value
	 * @param camera target camera
	 * @param encoding image's encode
	 */
	void get_image(cv_bridge::CvImagePtr& image, const E_Camera camera, const std::string& encoding = sensor_msgs::image_encodings::BGR8);
	/**
	 * @brief getter
	 * @param image return value
	 * @param camera target camera
	 * @param encoding image's encode
	 */
	void get_image(cv::Mat& image, const E_Camera camera, const std::string& encoding = sensor_msgs::image_encodings::BGR8);
private:
	// get active camera number
	unsigned int get_active_num();
	// get not active camera number
	bool get_not_active_device(std::list<E_Camera>& devicies);
	// get active device list
	bool get_active_device(std::list<E_Camera>& devicies);
	// get camera handler pointer from E_Camera
	CameraHandler* get_handler_pointer(const E_Camera camera);
};

#endif