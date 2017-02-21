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
#include <std_msgs/UInt16.h>

#include <ros_module/ROSInterface.hpp>

std::array<bool, ROSLEDController::LED_NUM> bound_pattern(int i){
    std::array<bool, ROSLEDController::LED_NUM> state;
    std::for_each(state.begin(), state.end(), [](bool& x){ x = false; });
    int target;
    if((i / ROSLEDController::LED_NUM) % 2 == 0) 
        target = i % ROSLEDController::LED_NUM;
    else    
        target = ROSLEDController::LED_NUM - i % ROSLEDController::LED_NUM;
    
    state[(target + 6) % ROSLEDController::LED_NUM] = true;
    return state;
};

std::array<bool, ROSLEDController::LED_NUM> alternative_pattern(int i){
    std::array<bool, ROSLEDController::LED_NUM> state;
    for(int j = 0; j < ROSLEDController::LED_NUM; ++ j){
        state[j] = j % 2 | !(j % 2 & i % 2);
    }
    return state;
};

//main entry point of this program
int main(int argc, char *argv[]){
    ROSLEDController led_controller;

    // Initialize 
    ros::init(argc, argv, "sample");
    led_controller.init();

    // Loop rate
    ros::Rate rate(30.0);
    for(int i = 0; i < 50 * ROSLEDController::LED_NUM; ++ i){
        led_controller.set_command(bound_pattern(i));
        rate.sleep();
    }

	return 0;
}
