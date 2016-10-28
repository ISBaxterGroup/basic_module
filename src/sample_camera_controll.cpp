//----------------------------------------------------------
// ROSJointController sample
//----------------------------------------------------------
#include <iostream>
#include <opencv2/core/core.hpp>
#include <iomanip>
#include <cmath>
#include <array>

// ROS
#include "ROSInterface.hpp"

//main entry point of this progra
int main(int argc, char *argv[])
{   

    ROSCameraController camera_controller;

    // Initialize 
    ros::init(argc, argv, "sample");
    ros::NodeHandle n;
    camera_controller.init();

    // Buffer
    // three data types can be use.
    // sensor_msgs::Image head, left, right;
    cv_bridge::CvImagePtr head( new cv_bridge::CvImage() );
    cv_bridge::CvImagePtr left( new cv_bridge::CvImage() );
    cv_bridge::CvImagePtr right( new cv_bridge::CvImage() );
    // cv::Mat head, left, right;

    // make window
    cv::namedWindow("Image_left", CV_WINDOW_AUTOSIZE|CV_WINDOW_FREERATIO);
    cv::namedWindow("Image_right", CV_WINDOW_AUTOSIZE|CV_WINDOW_FREERATIO);
    cv::namedWindow("Image_head", CV_WINDOW_AUTOSIZE|CV_WINDOW_FREERATIO);

    std::cout << "Two cameras that are not disabled will work!" << std::endl;
    std::cout << "r: Update image" << std::endl;

    std::cout << "z: disactivate left hand camera." << std::endl;
    std::cout << "x: disactivate right hand camera." << std::endl;
    std::cout << "c: disactivate head camera." << std::endl;

    std::cout << "a: activate left hand camera." << std::endl;
    std::cout << "s: activate right hand camera." << std::endl;
    std::cout << "d: activate head camera." << std::endl;

    std::cout << "q: quit." << std::endl;

    // [ms]
    const unsigned int wait_time(30);
    while(1){
        int key = cv::waitKey(wait_time);
        int mask = 0x0000FF;
        key = key & mask;

        if( camera_controller.new_data(ROSCameraController::E_Camera::E_LeftHand) ){
            camera_controller.get_image(left, ROSCameraController::E_Camera::E_LeftHand);
            cv::imshow("Image_left", left->image);
        }
        if( camera_controller.new_data(ROSCameraController::E_Camera::E_RightHand) ){
            camera_controller.get_image(right, ROSCameraController::E_Camera::E_RightHand);
            cv::imshow("Image_right", right->image);
        }
        if( camera_controller.new_data(ROSCameraController::E_Camera::E_Head) ){
            camera_controller.get_image(head, ROSCameraController::E_Camera::E_Head);
            cv::imshow("Image_head", head->image);
        }
        
        if(key == 'z') camera_controller.close_camera( ROSCameraController::E_Camera::E_LeftHand );
        if(key == 'x') camera_controller.close_camera( ROSCameraController::E_Camera::E_RightHand );
        if(key == 'c') camera_controller.close_camera( ROSCameraController::E_Camera::E_Head );

        if(key == 'a') camera_controller.open_camera( ROSCameraController::E_Camera::E_LeftHand );
        if(key == 's') camera_controller.open_camera( ROSCameraController::E_Camera::E_RightHand );
        if(key == 'd') camera_controller.open_camera( ROSCameraController::E_Camera::E_Head );

        if(key == 'q') break;
    }

}
