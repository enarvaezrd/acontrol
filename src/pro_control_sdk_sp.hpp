
#include "ros/ros.h"

#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <iostream>

#include <stdlib.h>
#include <stdio.h>
#include <vector>

#include <chrono>
#include <thread>

#include <dynamixel_workbench_msgs/DynamixelCommand.h>
#include <cstdlib>
#include <math.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JointState.h>
#include <control_msgs/FollowJointTrajectoryGoal.h>
#include <utility>
#define DEBUG_
#include "dynamixel_sdk/dynamixel_sdk.h" // Uses Dynamixel SDK library
using namespace std;

struct motor_params
{
    uint8_t ID;
    pair<int, int> motor_limits;
    double resolution;
    double offset = 0.0;
    double protocol_version = 2.0;
    int moving_threshold = 20;
    int torque_enable_addr = 562;    //pro H54 H42 , 512 for PRO H42P
    int led_red_addr = 563;          //513
    int goal_position_addr = 596;    //564
    uint16_t present_position_addr = 611; //580
    int velocity_limit_addr = 32;    //44
    int goal_position;
    int32_t current_position = 0;
    double current_angle = 0.0;
    int velocity_limit = 10;
    int reverse = 1; //reverse movement
};
