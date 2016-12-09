//----------------------------------------------------------
// Hook Controll sample
//----------------------------------------------------------
#include <iostream>

// Open GL
#include <GL/glut.h>
#include <GL/freeglut.h>

// Hook controller
#include "ROSHookController.hpp"
using namespace std;

// instanciate hook controller
ROSHookController hook_controller;

void show_description()
{
    cout << "-------------------------------------------------------" << endl;
    cout << "\\ : Switch right gripper state." << endl;
    cout << "/ : Switch left gripper state." << endl;
    cout << ". : Open right gripper with specifyed angle." << endl;
    cout << ", : Open left gripper with specifyed angle." << endl;
    cout << "\\ : Open both side gripper with specifyed angle." << endl;
    cout << "-------------------------------------------------------" << endl << endl;
};

//----------------------------------------------------------
// Call back method - key function
//----------------------------------------------------------
//keyfunc
void NormalKeyIn(unsigned char key, int x, int y){ 
    
    static ROSHookController::E_Left_State left_hand_state = ROSHookController::E_Left_State::E_Open;
    static ROSHookController::E_Right_State right_hand_state = ROSHookController::E_Right_State::E_Open;

    switch(key){ 
        case '/':
            // Reverce hand state
            left_hand_state = (left_hand_state == ROSHookController::E_Left_State::E_Open ? 
                ROSHookController::E_Left_State::E_Close :
                ROSHookController::E_Left_State::E_Open);

            // Publish hand state
            hook_controller.set_state(left_hand_state);
            cout << "/ pushed. switch gripper state" << endl;
            break;
        case '\\':
            // Reverce hand state
            right_hand_state = (right_hand_state == ROSHookController::E_Right_State::E_Open ? 
                ROSHookController::E_Right_State::E_Close :
                ROSHookController::E_Right_State::E_Open);

            // Publish hand state
            hook_controller.set_state(right_hand_state);
            cout << "\\ pushed. switch gripper state" << endl;
            break;
        case '.':
            // Publish hand state 
            hook_controller.set_state( ROSHookController::E_Right_State::E_Angle, ROSHookController::Degree(30.0) );
            cout << ". pushed. Gripper openes specifyed angle" << endl;
            break;
        case ',':
            // Publish hand state 
            hook_controller.set_state( ROSHookController::E_Left_State::E_Angle, ROSHookController::Radian( M_PI / 6.0 ) );
            cout << ". pushed. Gripper openes specifyed angle" << endl;
            break;
        case 'm':
            // Publish hand state 
            hook_controller.set_state( ROSHookController::Degree( 25.0 ), ROSHookController::Degree( 34.0 ) );
            cout << ". pushed. Gripper openes specifyed angle" << endl;
            break;
        case 27:
            exit(0); 
            break;

        default: 
            break; 
    }
    show_description();
}

//main entry point of this program
int main(int argc, char *argv[]){
    // ROS init
    ros::init(argc, argv, "sample_hook");
    hook_controller.init();

    // Open GL init
    glutInit(&argc, argv);
    
    // Set call back functoin
    glutCreateWindow(argv[0]);
    glutDisplayFunc([](){});
    glutKeyboardFunc(NormalKeyIn);
    
    show_description();
    //start main loop
    glutMainLoop();
	return 0;
}
