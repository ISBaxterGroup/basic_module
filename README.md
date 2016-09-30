# basic_module
The basic module for communicating with baxter easily

## RosIKClient
Class for utilizing the ikservice
### sample program
- sample_ik.cpp

## RosJointController
Class to be sent to the Baxter joint angles at a constant cycle
### sample program
- sample_arm.cpp
- sample_stop.cpp

## RosTFListener, RosTFBroadcaster
Class to listen and Broadcast transform.
### sample program
- sample_tf_listen.cpp
- sample_tf_broadcast.cpp

## RosLEDController
Class that manages the state of the LED that is attached to the head
### sample program
- sample_led.cpp

## RosHookController
Class that specifies the state of the gripper
### sample program
- sample_hook.cpp
