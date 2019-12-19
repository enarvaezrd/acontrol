#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#define WIRELESS_CONTROLLER

bool new_message_received = false;
sensor_msgs::Joy joystick_msg;
//---------------JOYSTICK------------------------
void Joy_Handler(const sensor_msgs::Joy &joystick)
{
    joystick_msg = joystick;
#ifndef WIRELESS_CONTROLLER
    double T2 = joystick_msg.axes[2];
    joystick_msg.axes[5] = T2;
#endif
    new_message_received = true;
}

const std::string keys =
    "{ help h ?       | false             | Show help command }"
    "{ altitude a1t   | 1.3               | UAV altitude for tracing task}";

int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker");
    ros::NodeHandle ros_node_handler("~");
    cv::CommandLineParser parser(argc, argv, keys);
    ros::Rate loop_rate(25);
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::Publisher pub_ugv_control = ros_node_handler.advertise<geometry_msgs::Twist>("/robot1/cmd_vel", 1); //UGV control topic

    ros::Subscriber sub_joystick = ros_node_handler.subscribe("/joy", 1, Joy_Handler);

    geometry_msgs::Twist command_msg;
    double Gain = 0.2;
    double GainAngular = Gain + 0.3;
    bool update_gain = true;
    while (ros::ok())
    {
        if (!new_message_received)
        {
            ros::spinOnce();
            loop_rate.sleep();
            continue;
        }
        if (joystick_msg.axes[5] == 0.0 || joystick_msg.axes[5] == 1.0)
        {
            command_msg.angular.x = 0.0;
            command_msg.angular.y = 0.0;
            command_msg.angular.z = 0.0;
            command_msg.linear.x = 0.0;
            command_msg.linear.y = 0.0;
            command_msg.linear.z = 0.0;
            pub_ugv_control.publish(command_msg);
            ros::spinOnce();
            loop_rate.sleep();
            continue;
        }
        if (update_gain)
        {
            if (joystick_msg.axes[6] == -1.0)
                Gain += 0.025;
            if (joystick_msg.axes[6] == 1.0)
                Gain -= 0.025;
            update_gain = false;
        }
        if (joystick_msg.axes[6] == 0.0)
            update_gain = true;

        if (Gain < 0.0)
            Gain = 0.0;

        GainAngular = -(Gain + 0.3);
        command_msg.angular.x = 0.0;
        command_msg.angular.y = 0.0;
        command_msg.angular.z = GainAngular * joystick_msg.axes[0];
        command_msg.linear.x = Gain * joystick_msg.axes[1];
        command_msg.linear.y = 0.0;
        command_msg.linear.z = 0.0;
        pub_ugv_control.publish(command_msg);

        ros::spinOnce();

        loop_rate.sleep();
    }
    return 0;
}
