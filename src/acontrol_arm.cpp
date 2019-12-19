#include "ros/ros.h"
#include "std_msgs/String.h"
#include <opencv2/opencv.hpp>
#include <sstream>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <sensor_msgs/Joy.h>
#include <chrono>
#define WIRELESS_CONTROLLER
float key_x = 0.0, key_az, key_y = 0.0, key_z, range, uav_GPS_height, landing_flag, height;
geometry_msgs::Pose markpose;
geometry_msgs::Twist Local_UAV_Position;
int state;

//---------------KEYBOARD------------------------
void Key_Handler(const geometry_msgs::Twist &key)
{
    key_az = key.angular.z;
    key_x = 0.5 * key.linear.x;
    key_y = 0.5 * key.linear.y;
    key_z = 0.5 * key.linear.z;
    landing_flag = key.angular.y;
}

//----------------SENSORS-------------------------
void sonar_Handler(const sensor_msgs::Range &rng)
{
    range = rng.range;
}

void GPS_Handler(const sensor_msgs::NavSatFix &gp)
{
    uav_GPS_height = gp.altitude;
}

//---------TOPIC HANDLER - FROM RRT NODE-----------
void Local_uav_Position_Handler(const geometry_msgs::Twist &local_uav_position)
{
    Local_UAV_Position = local_uav_position; //correction values
    //Local_UAV_Position.linear.y *= -1.0;
    return;
}

void Check_Limits(float value, float limit)
{
    if (value > limit)
        value = limit;
    if (value < -limit)
        value = -limit;
    return;
}
bool new_message_received = false;
sensor_msgs::Joy joystick_msg;
void Joy_Handler(const sensor_msgs::Joy &joystick)
{
    joystick_msg = joystick;

#ifndef WIRELESS_CONTROLLER
    double T2 = joystick_msg.axes[2];
    joystick_msg.axes[2] = joystick_msg.axes[3];
    joystick_msg.axes[3] = joystick_msg.axes[4];
    joystick_msg.axes[4] = joystick_msg.axes[5];
    joystick_msg.buttons[3] = joystick_msg.buttons[2];
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
    ros::Rate loop_rate(28.0);

    geometry_msgs::Twist ed_control;
    ed_control.linear.x = 0.0;
    ed_control.linear.y = 0.0;
    ed_control.linear.z = 0.8;
    ed_control.angular.y = 0.0;
    ed_control.angular.x = 0.0;
    ed_control.angular.z = 0.0;

    ros::Publisher pub_uav_control = ros_node_handler.advertise<geometry_msgs::Twist>("/robot2/cmd_vel", 1); //UAV control topic
    for (int i = 0; i < 40; i++)
    {
        pub_uav_control.publish(ed_control);
        ros::spinOnce();
        loop_rate.sleep();
    }

    ros::Subscriber sub_keyboard = ros_node_handler.subscribe("/turtlebot_teleop/cmd_vel", 1, Key_Handler); //keyboard topic
    ros::Subscriber sub_sonar = ros_node_handler.subscribe("/robot2/sonar_height", 1, sonar_Handler);
    ros::Subscriber sub_gps = ros_node_handler.subscribe("/robot2/fix", 1, GPS_Handler);
    ros::Subscriber sub_local_position = ros_node_handler.subscribe("/robot2/visual_local_guidance/uav_msg", 1, Local_uav_Position_Handler);
    ros::Subscriber sub_joystick = ros_node_handler.subscribe("/joy", 1, Joy_Handler); //Joystick

    float Altitude_Set_Point = parser.get<float>("altitude");

    float Altitude_Error = 0.0;
    int count = 0;
    float PI = 3.141592654;
    float Kp = 1.6;

    float Kd = 0.1;
    double error_x_old = 0.0;
    double error_y_old = 0.0;

    float Ki = 0.02; //0.02
    double Integral_x = 0.0;
    double Integral_y = 0.0;

    float Kp_altitude = 0.35;
    int iteration_indx = 0;

    auto time_start = std::chrono::high_resolution_clock::now();
    double Gain = 0.2; // initial lateral gain
    bool update_gain = true;
    int count_null_commands = 0;
    while (ros::ok())
    {

        iteration_indx++;
        Check_Limits(iteration_indx, 100);
        Altitude_Error = Altitude_Set_Point - uav_GPS_height;
        Check_Limits(Altitude_Error, 2.0);
        bool joystick_state = false;

        if (!new_message_received)
        {
            joystick_state = false; //just to bypass the first iterations when joy is not available
        }
        else if (joystick_msg.axes[5] == 0.0 || joystick_msg.axes[5] == 1.0)
        {
            joystick_state = false;
        }
        else
        {
            joystick_state = true;
        }

        if (!joystick_state)
        {
            ed_control.linear.x = 0.0;
            ed_control.linear.y = 0.0;
            ed_control.linear.z = 0.09 + key_z + (Altitude_Error * Kp_altitude);
            ed_control.angular.y = 0.0;
            ed_control.angular.x = 0.0;
            ed_control.angular.z = 0.0;

            Integral_x = 0.0;
            Integral_y = 0.0;

            error_x_old = 0.0;
            error_y_old = 0.0;
            pub_uav_control.publish(ed_control);
            ros::spinOnce();
            loop_rate.sleep();
            continue;
        }
        if (Local_UAV_Position.linear.z != 0.0)
        {
            Altitude_Set_Point = Local_UAV_Position.linear.z;
        }
        if (update_gain)
        {
            if (joystick_msg.axes[7] == 1.0)
                Gain += 0.02;
            if (joystick_msg.axes[7] == -1.0)
                Gain -= 0.02;
            update_gain = false;
        }
        if (joystick_msg.axes[7] == 0.0)
            update_gain = true;

        if (Gain < 0.0)
            Gain = 0.0;
        float joystick_x = Gain * joystick_msg.axes[3];
        float joystick_y = Gain * joystick_msg.axes[2];
        float Gain_z = -2.0;
        double angularJoyCommand = -joystick_msg.buttons[1] + joystick_msg.buttons[3];

        if (joystick_msg.axes[4] == 0.0 || joystick_msg.axes[4] == 1.0)
        {
            Gain_z = 0.0;
        }
        float joystick_z = Gain * Gain_z * (joystick_msg.axes[4] - 1.0);

        if (iteration_indx < 15) //warming up
        {
            ed_control.linear.x = key_x;
            ed_control.linear.y = key_y;
            ed_control.linear.z = 0.2 + (Altitude_Error * 0.35);
            ed_control.angular.y = 0.0;
            ed_control.angular.x = 0.0;
            ed_control.angular.z = key_az + angularJoyCommand;
        }
        else
        {
            auto time_end = std::chrono::high_resolution_clock::now();
            double elapsed_th = std::chrono::duration_cast<std::chrono::microseconds>(time_end - time_start).count()/1000000.0;
            time_start = std::chrono::high_resolution_clock::now();
            double derivative_x = (Local_UAV_Position.linear.x - error_x_old) / elapsed_th;
            double derivative_y = (Local_UAV_Position.linear.y - error_y_old) / elapsed_th;

            Integral_x += (Local_UAV_Position.linear.x * elapsed_th) ;
            Integral_y += (Local_UAV_Position.linear.y * elapsed_th) ;

            ed_control.linear.x = joystick_x + key_x + Kp * Local_UAV_Position.linear.x + Kd * derivative_x + Ki * Integral_x;
            ed_control.linear.y = joystick_y + key_y + Kp * Local_UAV_Position.linear.y + Kd * derivative_y + Ki * Integral_y;
            // std::cout << "-**- UAV ERROR x: " << Local_UAV_Position.linear.x << ", y: " << Local_UAV_Position.linear.y << std::endl;
            // std::cout << "-**- UAV CORRECCION x: " << ed_control.linear.x << ", y: " << ed_control.linear.y << std::endl;
            // std::cout << "-**- UAV Y VALUES: joy " << joystick_y << ", der: " << derivative_y << ", int: " << Integral_y << std::endl;
            ed_control.linear.z = key_z + (Altitude_Error * Kp_altitude) + joystick_z;
            ed_control.angular.y = 0.0;
            ed_control.angular.x = 0.0;
            ed_control.angular.z = key_az + angularJoyCommand;
            if (iteration_indx < 55)
                ed_control.linear.z = 0.01;

            error_x_old = Local_UAV_Position.linear.x;
            error_y_old = Local_UAV_Position.linear.y;
            if (Local_UAV_Position.linear.x == 0.0 && Local_UAV_Position.linear.x == 0.0)
            {
                count_null_commands++;
            }
            if (count_null_commands > 10)
            {
                count_null_commands = 0;
                Integral_x = 0.0;
                Integral_y = 0.0;
            }
        }

        pub_uav_control.publish(ed_control);
        //pub3.publish(sum_control);
        ros::spinOnce();

        loop_rate.sleep();
        ++count;
    }
    return 0;
}
