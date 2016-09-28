/**
 * @file ROSTFListener.cpp
 * @brief 
 *  ROSTFListener sample
 * @author Iwase
 * @date 2016.9.28.
 */
//----------------------------------------------------------
// ROSTFListener sample
//----------------------------------------------------------
#include <iostream>
#include <cstdio>
#include <cmath>
#include <array>
#include <string>
// ROS
#include <ros/ros.h>
#include "ROSTFListener.hpp"


//main entry point of this program
int main(int argc, char *argv[]){

    const std::string src("base");
    const std::string dst("desk");
    ROSTFListener tf_listener(src, dst);

    // Initialize 
    ros::init(argc, argv, "sample_tf");
    tf_listener.init();

    std::array<double, 3> trans; 
    std::array<double, 4> rot;

    // 10 hz
    ros::Rate timer(10.0);

    std::cout << "Enter to continue, q to quit." << std::endl;
    std::cout << "input -> " ;
    while(auto c = getchar() != 'q'){
        
        bool valid = tf_listener.get_transform(trans, rot);
        
        // latest transform
        std::cout << "--------------------------------------" << std::endl;
        std::cout << "--- current base to desk transform ---" << std::endl;
        std::cout << "--------------------------------------" << std::endl;
        std::cout << "validity : " << std::boolalpha << valid << std::endl;
        std::cout << "trans    : " << trans[0] << " " << trans[1] << " " << trans[2] << std::endl;
        std::cout << "rot      : " << rot[0] << " " << rot[1] << " " << rot[2] << " " << rot[3] << std::endl;
        std::cout << "--------------------------------------" << std::endl;
        std::cout << "Enter to continue, q to quit." << std::endl;
        std::cout << "input -> " ;
        // Sleep 1/rate [sec]
        timer.sleep();
    }
    
	
    return 0;
}
