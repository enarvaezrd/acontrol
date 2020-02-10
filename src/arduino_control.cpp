/*
 *Node designed to reduce frequency of the publication of flags for arduino
 *Eduardo Narvaez
*/

#include "ros/ros.h"
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int8.h>
#include <iostream>
bool docking_flag_received = false; // signal to lock
bool EM_flag_received = false;      // start of docking

void Joy_Handler(const sensor_msgs::Joy::ConstPtr &joystick)
{
    // ROS_INFO("I heard: [%d]", joystick->buttons[11]);
    //ROS_INFO("I heard: [%d]", joystick->buttons[4]);
    // std::cout<<"flag : "<<joystick->buttons[11]<<","<<joystick->buttons[4]<<std::endl;
    if (joystick->buttons[11] == 1.0)
    {
        if (docking_flag_received)
        {
            docking_flag_received = false;
        }
        else
        {
            docking_flag_received = true;
        }
    }
    if (joystick->buttons[4] == 1.0)
    {
        if (EM_flag_received)
        {
            EM_flag_received = false;
        }
        else
        {
            EM_flag_received = true;
        }
    }
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "arduino_control");

    ros::NodeHandle ros_node_handler;
    ros::Rate loop_rate(4);
    ros::Subscriber sub_joystick = ros_node_handler.subscribe("/joy", 1, Joy_Handler); //Joystick

    ros::Publisher arduino_pub = ros_node_handler.advertise<std_msgs::Int8>("/robot1/arduino_control/flags_msg", 1); //commands for the UAV

    std_msgs::Int8 pub_msg;
    pub_msg.data = 0;
    arduino_pub.publish(pub_msg);
    bool msg_sent = false;
    bool oldDcking = false, oldEM = false;
    while (ros::ok())
    {
        // std::cout<<"flag : "<<EM_flag_received<<std::endl;

        if (oldDcking == docking_flag_received && oldEM == EM_flag_received)
        {
            loop_rate.sleep();
            ros::spinOnce();
            continue;
        }
        oldDcking = docking_flag_received;
        oldEM = EM_flag_received;
        if (EM_flag_received)
        {
            if (!docking_flag_received)
            {
                pub_msg.data = 1;
                arduino_pub.publish(pub_msg); // docking completion
            }
            else
            {
                pub_msg.data = 2;
                arduino_pub.publish(pub_msg); //just EM activated
            }
        }
        else 
        {

            if (!docking_flag_received)
            {
                pub_msg.data = 3;
                arduino_pub.publish(pub_msg); // docking completion
            }
            else
            {
                pub_msg.data = 0;
                arduino_pub.publish(pub_msg); //just EM activated
            }
        }

        loop_rate.sleep();
        ros::spinOnce();
    }
}
