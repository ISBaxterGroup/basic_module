/**
 * @file sample_tf_broadcaster.cpp
 * @brief 
 *  ROSTFBroadcaster sample
 * @author Iwase
 * @date 2016.9.29.
 */
//----------------------------------------------------------
// ROSTFBroadcaster sample
//----------------------------------------------------------
#include <iostream>
#include <thread>
#include <cstdio>
#include <cmath>
#include <array>
#include <string>
// ROS
#include <ros/ros.h>
#include "ROSTFBroadcaster.hpp"

//----------------------------------------------------------
// mutex
//----------------------------------------------------------
std::mutex mtx;
//----------------------------------------------------------
// get key loop
//----------------------------------------------------------
void get_key_loop(bool& loop_continue){ 
    
    do {
        std::cout << "broadcasting world to box transform!" << std::endl;
        std::cout << "input q to quit -> ";
    } while( getchar() != 'q');

    mtx.lock();
    loop_continue = false;
    mtx.unlock();
}

//main entry point of this program
int main(int argc, char *argv[]){
    // Start get_key loop
    std::thread get_key_thread;
    bool loop_continue_shere(true);
    try {
        get_key_thread = std::thread(get_key_loop, std::ref(loop_continue_shere));
    }
    catch (std::system_error& e) {
        std::cout << e.what() << std::endl;
    }

    // Initialize 
    ros::init(argc, argv, "sample_tf_broadcast");
    const std::string src("world");
    const std::string dst("box");
    ROSTFBroadcaster broadcaster(src, dst);
    broadcaster.init();

    ros::Rate timer(10.0);
    bool loop_continue(true);
    unsigned int num = 0;
    unsigned int division_num = 100;
    std::array<double, 3> trans{ {0, 0, 0} }; 
    std::array<double, 4> rot{ {1, 0, 0, 0} };

    while(loop_continue){

        // rotate coordinate system by axis and angle
        double th = num * (2 * M_PI / division_num); 
        num = (num + 1) % division_num;
        tf::Quaternion q( tf::Vector3(0, 0.5, 0.5), th );
        for(unsigned int i = 0; i < 4; ++ i) rot[i] = q[i];

        // update transform
        broadcaster.set_transform(trans, rot);

        mtx.lock();
        loop_continue = loop_continue_shere;
        mtx.unlock();

        timer.sleep();
    }
    
	get_key_thread.join();
    return 0;
}
