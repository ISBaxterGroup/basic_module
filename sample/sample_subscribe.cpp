/**
 * @file sample_subscribe.cpp
 * @brief 
 *  ROSSubscriberInterface sample.
 *  Subscribe /chatter topic and display data.
 *  Run this program and type the following on another command line.
 *  > rostopic pub /chatter std_msgs/String "Message Here"
 * @author Iwase
 * @date 2017.1.5.
 */
//----------------------------------------------------------
// include
//----------------------------------------------------------
#include <iostream>
#include <string>

#include <GL/glut.h>
#include <GL/freeglut.h>

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <GL/glut.h>
#include <GL/freeglut.h>

#include <ros_module/ROSInterface.hpp>

// instanciate ROSSubscriberInterface
ROSSubscriberInterface<std_msgs::String> sub_interface("/chatter");
//----------------------------------------------------------
// Call back method - Display
//----------------------------------------------------------
void Display()
{  
    if(sub_interface.new_data()){
        // Buffer
        std_msgs::String str;
        // Copy the latest data to str.
        sub_interface.get(str);

        std::cout << "data: " << str.data << std::endl;
    }
    // 60Hz timer
    ros::Rate loop_rate(60);
    loop_rate.sleep();

};
//----------------------------------------------------------
// Call back method - idle function
//----------------------------------------------------------
void Idle(){
    glutPostRedisplay();
}
//----------------------------------------------------------
// Call back method - key function
//----------------------------------------------------------
//keyfunc
void NormalKeyIn(unsigned char key, int x, int y){ 
    switch(key){ 
        case 27:
            exit(0); 
            break;
        case 'r':
            // call DisplayFunc. 
            glutPostRedisplay();
            break;
        default: 
            std::cout << key << " pushed." << std::endl << "ESC : quit." << std::endl << "r : reload." << std::endl;
            break; 
    }
}

//main entry point of this program
int main(int argc, char *argv[]){
    ros::init(argc, argv, "sample_hook");
    // Initialize subscriber
    sub_interface.init();

    glutInit(&argc, argv);
    glutCreateWindow(argv[0]);
    glutDisplayFunc(Display);
    glutIdleFunc(Idle);
    glutKeyboardFunc(NormalKeyIn);
    glutMainLoop();
    return 0;
}
