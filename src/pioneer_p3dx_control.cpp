#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int8.h>
#include <iostream>

bool new_message_received = false;
sensor_msgs::Joy joystick_msg;
//---------------JOYSTICK------------------------
void Joy_Handler(const sensor_msgs::Joy &joystick)
{
    joystick_msg = joystick;
    new_message_received = true;
}
bool tracking_docking_state = false;
void Docking_State_Handler(const std_msgs::Int8 &docking_state_)
{
    // ROS_INFO("I heard: [%d]", joystick->buttons[11]);
    //ROS_INFO("I heard: [%d]", joystick->buttons[4]);
    // std::cout<<"flag : "<<joystick->buttons[11]<<","<<joystick->buttons[4]<<std::endl;
    if (docking_state_.data != 0.0 && docking_state_.data != 5.0)
    {
        tracking_docking_state = true;
    }
    else
    {
        tracking_docking_state = false;
    }
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

    ros::Subscriber sub_docking_state = ros_node_handler.subscribe("/docking_state", 1, Docking_State_Handler);       //Joystick
    ros::Publisher pub_ugv_control = ros_node_handler.advertise<geometry_msgs::Twist>("/RosAria_control/cmd_vel", 1); //UGV control topic

    ros::Subscriber sub_joystick = ros_node_handler.subscribe("/joy", 1, Joy_Handler);

    geometry_msgs::Twist command_msg;

    double Initial_Gain = 0.07;
    double Gain = Initial_Gain;
    bool update_gain = true;
    while (ros::ok())
    {
        if (!new_message_received)
            continue;
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

        if (tracking_docking_state)
        {
            Gain = 0.02;
        }
        else
        {
            if (Gain < Initial_Gain)
                Gain = Initial_Gain;
        }

        if (update_gain)
        {
            if (joystick_msg.axes[6] == -1.0)
                Gain += 0.015;
            if (joystick_msg.axes[6] == 1.0)
                Gain -= 0.015;
            update_gain = false;
        }
        if (joystick_msg.axes[6] == 0.0)
            update_gain = true;

        if (Gain < 0.0)
            Gain = 0.001;

        command_msg.angular.x = 0.0;
        command_msg.angular.y = 0.0;
        command_msg.angular.z = 1.5 * Gain * joystick_msg.axes[0];
        command_msg.linear.x = Gain * joystick_msg.axes[1];
        command_msg.linear.y = 0.0;
        command_msg.linear.z = 0.0;
        pub_ugv_control.publish(command_msg);

        ros::spinOnce();

        loop_rate.sleep();
    }
    return 0;
}
