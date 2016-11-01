//----------------------------------------------------------
// ROSJointController sample
//----------------------------------------------------------
#include <iostream>
#include <iomanip>
#include <cmath>
#include <array>

// ROS
#include <ros/ros.h>
#include "ROSJointController.hpp"

//main entry point of this program
int main(int argc, char *argv[]){
    ROSJointController joint_controller;

    // Initialize 
    ros::init(argc, argv, "sample");
    joint_controller.init();

    const double AMPLITUDE(M_PI / 10);
    const double D_TH(0.1);
    double th(0.0);

    // Loop rate
    ros::Rate rate(10.0);
    const unsigned int loop_num(300);
    for(unsigned int i = 0; i < loop_num; ++ i){
        th = D_TH * i;

        // Angle in joints
        std::array<double, 7> joints_l { AMPLITUDE / 2 * sin(th / 5) - AMPLITUDE, AMPLITUDE * sin(th) -AMPLITUDE, 0, 2 * AMPLITUDE * sin(-th) + 2 * AMPLITUDE, 0, 4 * AMPLITUDE * sin(th) - 2 * AMPLITUDE, 0};
        std::array<double, 7> joints_r { AMPLITUDE / 2 * sin(-th / 5) + AMPLITUDE, AMPLITUDE * sin(th) -AMPLITUDE, 0, 2 * AMPLITUDE * sin(-th) + 2 * AMPLITUDE, 0, 4 * AMPLITUDE * sin(th) - 2 * AMPLITUDE, 0};

        // Set angle
        joint_controller.set_command(ROSJointController::LeftJCommand(joints_l));
        joint_controller.set_command(ROSJointController::RightJCommand(joints_r));

        // Get angle
        std::array<double, ROSJointController::JOINTS_NUM> curr_left_state, curr_right_state;
        joint_controller.get_left_joint_angle(curr_left_state);
        joint_controller.get_right_joint_angle(curr_right_state);

        // Show current state of joint got from ROS
        auto show_val = [](double a){ std::cout  << std::fixed << std::setprecision(4) << a << " "; };

        std::cout << "left : [ ";
        for(auto val : curr_left_state) show_val(val);
        std::cout << "]" << std::endl;
        
        std::cout << "right : [ ";
        for(auto val : curr_right_state) show_val(val);
        std::cout << "]" << std::endl;
        
        // Sleep 1/rate [sec]
        rate.sleep();
    }

	return 0;
}
