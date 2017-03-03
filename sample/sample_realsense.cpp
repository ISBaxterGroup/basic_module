//----------------------------------------------------------
// ROSIKClient sample
//----------------------------------------------------------
#include <iostream>
#include <memory>
#include <cmath>
#include <array>
// ROS
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <image_transport/image_transport.h>
// msgs
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <basic_module/Deproject.h>
// ros module
#include <ros_module/ROSInterface.hpp>

struct ROSConnect{
    static ROSJointController joint_controller;
    static ROSIKClient ik_client;
    static ROSTFListener tf_base_camera;
    static ROSTFListener tf_base_rightgripper;
    static ROSTFListener tf_rendgipper2_rightgipper;
    static ROSSubscriberInterface<sensor_msgs::ImageConstPtr> image_subscriber;
    static ROSServiceInterface<basic_module::Deproject> deproject_service;
    static inline void init()
    {
        joint_controller.init();
        ik_client.init();
        tf_base_rightgripper.init();
        tf_rendgipper2_rightgipper.init();
        tf_base_camera.init();
        image_subscriber.init();
        deproject_service.init();
    };
};

ROSJointController ROSConnect::joint_controller;
ROSIKClient ROSConnect::ik_client;
ROSTFListener ROSConnect::tf_base_rightgripper("base", "right_endgripper2");
ROSTFListener ROSConnect::tf_rendgipper2_rightgipper("right_endgripper2", "right_gripper");
ROSTFListener ROSConnect::tf_base_camera("base", "camera");
ROSSubscriberInterface<sensor_msgs::ImageConstPtr> ROSConnect::image_subscriber("rs_camera/image_color");
ROSServiceInterface<basic_module::Deproject> ROSConnect::deproject_service("deproject");
std::array<double, 3> to_right_gripper_target(const std::array<double, 3>& right_endgripper2)
{
    tf::Vector3 shift;
    {
        tf::Quaternion q;
        tf::Vector3 t;
        ROSConnect::tf_rendgipper2_rightgipper.get_transform(t, q);
        tf::Transform tr(q.inverse());
        shift = tr(t);
    }
    {
        tf::Quaternion q;
        ROSConnect::tf_base_rightgripper.get_rotation(q);
        tf::Transform tr(q);
        shift = tr(shift);
    }
    std::array<double, 3> res;
    res[0] = right_endgripper2[0] + shift[0];
    res[1] = right_endgripper2[1] + shift[1];
    res[2] = right_endgripper2[2] + shift[2];
    return res;
};

void move_right_hand(const std::array<double, 3>& target_pos,const std::array<double, 4>& target_rot)
{
    // IK solve mode (http://sdk.rethinkrobotics.com/wiki/API_Reference#Inverse_Kinematics_Solver_Service)
    const unsigned int SEED_AUTO    = 0;
    const unsigned int SEED_USER    = 1;
    const unsigned int SEED_CURRENT = 2;
    const unsigned int SEED_NS_MAP  = 3;

    // Call ik service
    ROSIKClient::RightRequest req_r(SEED_CURRENT, target_pos, target_rot);
    if(ROSConnect::ik_client.call(req_r)){
        std::array<double, 7> joints_r = ROSConnect::ik_client.get_right_joints();
        // Set angle
        ROSConnect::joint_controller.set_command(ROSJointController::RightJCommand(joints_r));
    }
    else std::cout << "ik can't solve!" << std::endl;
};

//! transform
std::array<double, 3> transform_camera_to_base(const std::array<double, 3>& a)
{
    assert(ROSConnect::tf_base_camera.valid_data());
    return ROSConnect::tf_base_camera.transform(a);
};
//! camera image to 3d point
std::array<double, 3> deproject(const std::array<double, 2>& request)
{
    basic_module::Deproject srv;
    srv.request.u = request[0];
    srv.request.v = request[1];
    // call deproject service
    std::cout << "deproject - calling deproject_service." << std::endl;
    if(!ROSConnect::deproject_service.call(srv)){
        std::cout << "service return false" << std::endl;
        exit(EXIT_FAILURE);
    }
    return std::array<double, 3>{srv.response.x, srv.response.y, srv.response.z};
};
//! Return clicked point.
std::array<double, 2> get_pixel(){
    cv::namedWindow("captured image", cv::WINDOW_AUTOSIZE);
    std::cout << "press any key to capture image" << std::endl;
    cv::waitKey(0);
    
    // Get latest image in topic
    sensor_msgs::ImageConstPtr sm_image;
    if(ROSConnect::image_subscriber.new_data()){
        ROSConnect::image_subscriber.get(sm_image);
    }
    else{
        std::cout << "There are no image in topic." << std::endl;
        exit(EXIT_FAILURE);
    }

    // convert sensor_msgs to cv::Mat
    cv_bridge::CvImagePtr cv_ptr;
    try{
        cv_ptr = cv_bridge::toCvCopy(sm_image, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        exit(EXIT_FAILURE);
    }
    cv::Mat image = cv_ptr->image;  

    // click event 
    cv::Vec2d cap1;
    auto func = [](int event, int x, int y, int flags, void* param){
            cv::Vec2d* p_vec = static_cast<cv::Vec2d*>(param);
            switch (event){
                case cv::EVENT_LBUTTONDOWN:
                    std::cout << "captured : " << x << " " << y << std::endl;
                    (*p_vec)[0] = x;
                    (*p_vec)[1] = y;
                    break;
                default:
                    break;
            }
        };

    // User click time.
    std::cout << "click to capture right hand target point! and press any key" << std::endl;
    cv::namedWindow("cap1", cv::WINDOW_AUTOSIZE);
    cv::setMouseCallback("cap1", func, (void *)&cap1);
    imshow("cap1", image);
    cv::waitKey();
    cv::destroyWindow("cap1");

    return std::array<double, 2>{cap1[0], cap1[1]};
};

//main entry point of this program
int main(int argc, char *argv[]){

    const tf::Quaternion init_rot(-0.139774, 0.989892, -0.00932151, 0.0221204);
    const tf::Vector3 init_trans(0.578843, -0.182984, 0.113031);

    // Copy to array
    std::array<double, 3> trans {init_trans[0], init_trans[1], init_trans[2]};
    std::array<double, 4> rot {init_rot[0], init_rot[1], init_rot[2], init_rot[3]};

    // Initialize 
    ros::init(argc, argv, "sample");
    ROSConnect::init();

    std::cout << "Enter to get color image, q to quit." << std::endl;
    std::cout << "input -> " ;

    while(auto c = getchar() != 'q'){
        
        std::array<double, 2> image_point = get_pixel();
        std::array<double, 3> deprojected_point = deproject(image_point);
        std::array<double, 3> base_coordinates_point = transform_camera_to_base(deprojected_point);
        
        move_right_hand(to_right_gripper_target(base_coordinates_point), rot);

        std::cout << "Enter to get color image, q to quit." << std::endl;
        std::cout << "input -> " ;
    }

	return 0;
}
