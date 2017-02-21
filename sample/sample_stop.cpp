//----------------------------------------------------------
// ROSJointController sample
//----------------------------------------------------------
#include <iostream>
#include <cmath>
#include <array>

// ROS
#include <ros/ros.h>
#include <ros_module/ROSInterface.hpp>

//main entry point of this program
int main(int argc, char *argv[]){
    ROSJointController joint_controller;

    // Initialize 
    ros::init(argc, argv, "sample");
    joint_controller.init();

    const double AMPLITUDE(M_PI / 10);
    double th(0.0);

    // Loop rate
    ros::Rate rate(10.0);
    for(int i = 0; i < 300; ++ i){
        th += 0.1;
        // Angle in joints
        //std::array<double, 7> joints_l { AMPLITUDE / 2 * sin(th / 5) - AMPLITUDE, AMPLITUDE * sin(th) -AMPLITUDE, 0, 2 * AMPLITUDE * sin(-th) + 2 * AMPLITUDE, 0, 4 * AMPLITUDE * sin(th) - 2 * AMPLITUDE, 0};
        std::array<double, 7> joints_l { -0.4912573467590332, 0.026844663757324222,-1.1915195756286623, 2.2929177800720217, -1.2547962830566408, 1.7533400385498048, 2.0616701765625};
        std::array<double, 7> joints_r { -0.5, AMPLITUDE * sin(th) -AMPLITUDE, 0, 2 * AMPLITUDE * sin(-th) + 2 * AMPLITUDE, 0, 4 * AMPLITUDE * sin(th) - 2 * AMPLITUDE, 0};

        // Set angle
        joint_controller.set_command(ROSJointController::LeftJCommand(joints_l));
        joint_controller.set_command(ROSJointController::RightJCommand(joints_r));

        // Sleep 1/rate [sec]
        rate.sleep();
    }

	return 0;
}
