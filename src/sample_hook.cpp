//----------------------------------------------------------
// Hook Controll sample
//----------------------------------------------------------
#include <iostream>

// Open GL
#include <GL/glut.h>
#include <GL/freeglut.h>

// Hook controller
#include "ROSHookController.hpp"
ROSHookController hook_controller;
using namespace std;
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
            hook_controller.set_command(left_hand_state);
            cout << "/" << endl;
            break;
        case '\\':
            // Reverce hand state
            right_hand_state = (right_hand_state == ROSHookController::E_Right_State::E_Open ? 
                ROSHookController::E_Right_State::E_Close :
                ROSHookController::E_Right_State::E_Open);

            // Publish hand state
            hook_controller.set_command(right_hand_state);
            cout << "\\" << endl;
            break;

        case 27:
            exit(0); 
            break;

        default: 
            break; 
    }
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

    //start main loop
    glutMainLoop();
	return 0;
}
