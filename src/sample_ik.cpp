//----------------------------------------------------------
// ROSIKClient sample
//----------------------------------------------------------
#include <iostream>
#include <cmath>
#include <array>

// ROS
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>

#include "ROSJointController.hpp"
#include "ROSIKClient.hpp"

//main entry point of this program
int main(int argc, char *argv[]){

    const tf::Quaternion init_q_l(0.160082, 0.986876, 0.0126797, 0.0169921);
    const tf::Quaternion init_q_r(-0.139774, 0.989892, -0.00932151, 0.0221204);
    const tf::Vector3 init_x_l(0.582458, 0.194376, 0.119101);
    const tf::Vector3 init_x_r(0.578843, -0.182984, 0.113031);

    ROSJointController joint_controller;
    ROSIKClient ik_client;

    // Initialize 
    ros::init(argc, argv, "sample");
    joint_controller.init();
    ik_client.init();

    ros::Rate rate(20.0);

    // Rotate angle
    double angle(0);
    double omg = 0.01;

    for(int i = 0; i < 300; ++ i){
        if(angle > 0.5) omg = -0.01;
        if(angle < -0.5) omg = 0.01;
        angle += omg;

        // Rotate axis 
        tf::Vector3 axis_l(0, 1, 0);
        tf::Vector3 axis_r(1, 0, 0);
        tf::Quaternion rot_l(axis_l, angle);
        tf::Quaternion rot_r(axis_r, angle);

        tf::Quaternion q_l(init_q_l);
        tf::Quaternion q_r(init_q_r);

        // Coordinate transformation left_endgripper2 -> left_wrist
        q_l *= rot_l;
        q_r *= rot_r;

        tf::Vector3 endgripper_length(0, 0, -0.12);

        tf::Vector3 x_l = init_x_l + tf::Transform(q_l)(endgripper_length);
        tf::Vector3 x_r = init_x_r + tf::Transform(q_r)(endgripper_length);

        // Copy to array
        std::array<double, 3> arr_x_l {x_l[0], x_l[1], x_l[2]};
        std::array<double, 3> arr_x_r {x_r[0], x_r[1], x_r[2]};
        std::array<double, 4> arr_q_l {q_l[0], q_l[1], q_l[2], q_l[3]};
        std::array<double, 4> arr_q_r {q_r[0], q_r[1], q_r[2], q_r[3]};

        // IK solve mode (http://sdk.rethinkrobotics.com/wiki/API_Reference#Inverse_Kinematics_Solver_Service)
        const unsigned int SEED_AUTO    = 0;
        const unsigned int SEED_USER    = 1;
        const unsigned int SEED_CURRENT = 2;
        const unsigned int SEED_NS_MAP  = 3;

        // Call ik service
        ROSIKClient::LeftRequest req_l(SEED_CURRENT, arr_x_l, arr_q_l);
        ROSIKClient::RightRequest req_r(SEED_CURRENT, arr_x_r, arr_q_r);

        if(!ik_client.call(req_l))  break;
        std::array<double, 7> joints_l = ik_client.get_left_joints();
        
        if(!ik_client.call(req_r)) break;
        std::array<double, 7> joints_r = ik_client.get_right_joints();

        // Set angle
        joint_controller.set_command(ROSJointController::LeftJCommand(joints_l));
        joint_controller.set_command(ROSJointController::RightJCommand(joints_r));

        // Sleep 1/rate [sec]
        rate.sleep();
    }

	return 0;
}
