#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include <opencv2/opencv.hpp>
#include <sstream>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <sensor_msgs/Joy.h>
#include <chrono>
#include <thread>

float key_x = 0.0, key_az, key_y = 0.0, key_z, range, uav_GPS_height, landing_flag, height;
geometry_msgs::Pose markpose;
geometry_msgs::Twist Local_UAV_Position, empty_position;
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
bool new_local_pos = false;
void Local_uav_Position_Handler(const geometry_msgs::Twist &local_uav_position)
{
    Local_UAV_Position = local_uav_position; //correction values
                                             //Local_UAV_Position.linear.y *= -1.0;
    new_local_pos = true;
    return;
}

float Check_Limits(float value, float limit)
{
    if (value > limit)
        value = limit;
    if (value < -limit)
        value = -limit;
    return value;
}

bool new_message_received = false;
sensor_msgs::Joy joystick_msg;
void Joy_Handler(const sensor_msgs::Joy &joystick)
{
    joystick_msg = joystick;
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
    ros::Rate loop_rate(25.0);

    geometry_msgs::Twist ed_control;
    ed_control.linear.x = 0.0;
    ed_control.linear.y = 0.0;
    ed_control.linear.z = 0.8;
    ed_control.angular.y = 0.0;
    ed_control.angular.x = 0.0;
    ed_control.angular.z = 0.0;

    ros::Publisher pub_uav_control = ros_node_handler.advertise<geometry_msgs::Twist>("/bebop/cmd_vel", 1); //UAV control topic
    ros::Publisher pub_takeoff = ros_node_handler.advertise<std_msgs::Empty>("/bebop/takeoff", 1);
    ros::Publisher pub_land = ros_node_handler.advertise<std_msgs::Empty>("/bebop/land", 1);
    ros::Publisher pub_reset = ros_node_handler.advertise<std_msgs::Empty>("/bebop/reset", 1);
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
    float Kp = 0.08;

    float Kd = 0.03;
    double error_x_old = 0.0;
    double error_y_old = 0.0;

    float Ki = 0.002; //0.02
    double Integral_x = 0.0;
    double Integral_y = 0.0;

    float Kp_altitude = 0.35;
    int iteration_indx = 0;

    auto time_start = std::chrono::high_resolution_clock::now();
    double Gain = 0.1;
    bool update_gain = true;
    int count_null_commands = 0;
    bool take_off = false;
    bool reset_state = false;
    float max_control_value = 0.2;
    while (ros::ok())
    {

        if (new_local_pos == false)
        {
            Local_UAV_Position = empty_position;
            //Integral_x=0.0;
            //Integral_y=0.0;
        }
        else
        {
            new_local_pos = false;
        }
        float Gain_z_command = 0.0;
        iteration_indx++;
        iteration_indx = Check_Limits(iteration_indx, 100);

        if (reset_state)
        {
            if (joystick_msg.buttons[6] == 1)
            {
                reset_state = false;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            ros::spinOnce();
            loop_rate.sleep();
            continue;
        }
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
            if (!take_off)
            {
                std_msgs::Empty Empty_msg;
                pub_takeoff.publish(Empty_msg);
                take_off = true;
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            }

            joystick_state = true;
        }

        if (!joystick_state)
        {
            ed_control.linear.x = 0.0;
            ed_control.linear.y = 0.0;
            ed_control.linear.z = 0.0;
            ed_control.angular.y = 0.0;
            ed_control.angular.x = 0.0;
            ed_control.angular.z = 0.0;
            pub_uav_control.publish(ed_control);
            ros::spinOnce();
            loop_rate.sleep();
            continue;
        }
        if (joystick_msg.buttons[6] == 1) //emergency mode, will fall, take care
        {
            std_msgs::Empty Empty_msg;
            pub_reset.publish(Empty_msg);
            reset_state = true;
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            ros::spinOnce();
            loop_rate.sleep();
            continue;
        }

        if (joystick_msg.buttons[7] == 1)
        {
            std_msgs::Empty Empty_msg;
            pub_land.publish(Empty_msg);
            take_off = false;
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            ros::spinOnce();
            loop_rate.sleep();
            continue;
        }
        if (update_gain)
        {
            if (joystick_msg.axes[7] == 1.0)
            {
                Gain += 0.01;
            }
            if (joystick_msg.axes[7] == -1.0)
            {
                Gain -= 0.01;
            }
            update_gain = false;
        }
        if (joystick_msg.axes[7] == 0.0)
            update_gain = true;

        if (Gain < 0.0) //uncomment when gain is variable for xy
            Gain = 0.0;
        float joystick_x = Gain * joystick_msg.axes[3]; //fixed gain
        float joystick_y = Gain * joystick_msg.axes[2];
        float Gain_z = 0.0;
        double angularJoyCommand = -joystick_msg.buttons[1] + joystick_msg.buttons[3];

        if (joystick_msg.axes[4] == 0.0 || joystick_msg.axes[4] == 1.0)
        {
            Gain_z = 0.0;
        }
        else
        {
            Gain_z = 1.0;
        }
        if (joystick_msg.buttons[14] == 1.0)
        {
            Gain_z = -1.0;
        }
        if (Local_UAV_Position.linear.z > 0) //Position linear z is the requested altitude
        {
            Gain_z_command = 1.0;
        }
        if (Local_UAV_Position.linear.z < 0)
        {
            Gain_z_command = -1.0;
        }
        float joystick_z = Gain * (Gain_z); // Gain * (Gain_z_command + Gain_z);

        auto time_end = std::chrono::high_resolution_clock::now();
        auto elapsed_th = std::chrono::duration_cast<std::chrono::microseconds>(time_end - time_start);
        time_start = std::chrono::high_resolution_clock::now();
        double derivative_x = 1000000.0 * (Local_UAV_Position.linear.x - error_x_old) / elapsed_th.count();
        double derivative_y = 1000000.0 * (Local_UAV_Position.linear.y - error_y_old) / elapsed_th.count();

        Integral_x += (Local_UAV_Position.linear.x * elapsed_th.count()) / 1000000.0;
        Integral_y += (Local_UAV_Position.linear.y * elapsed_th.count()) / 1000000.0;

        ed_control.linear.x = joystick_x + Kp * Local_UAV_Position.linear.x + Kd * derivative_x + Ki * Integral_x;
        ed_control.linear.y = joystick_y + Kp * Local_UAV_Position.linear.y + Kd * derivative_y + Ki * Integral_y;

        //std::cout << "-**- UAV ERROR x: " << Local_UAV_Position.linear.x << ", y: " << Local_UAV_Position.linear.y << std::endl;
        //std::cout << "-**- UAV CORRECCION x: " << ed_control.linear.x << ", y: " << ed_control.linear.y << std::endl;

        ed_control.linear.z = joystick_z;
        ed_control.angular.y = 0.0;
        ed_control.angular.x = 0.0;
        ed_control.angular.z = angularJoyCommand;

        std::cout << "-**- UAV JOY VALUES    : x" << joystick_x << ", y: " << joystick_y << ", z: " << joystick_z << ", angular: " << angularJoyCommand << std::endl;

        std::cout << "-**- UAV CONTROL VALUES: x" << ed_control.linear.x << ", y: " << ed_control.linear.y << ", z: " << ed_control.linear.z << ", angular: " << ed_control.angular.z << std::endl;
        // if (iteration_indx < 55)
        // ed_control.linear.z = 0.01;

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
        ed_control.linear.x = Check_Limits(ed_control.linear.x, max_control_value);
        ed_control.linear.y = Check_Limits(ed_control.linear.y, max_control_value);
        ed_control.linear.z = Check_Limits(ed_control.linear.z, 5.0 * max_control_value);

        pub_uav_control.publish(ed_control);
        //pub3.publish(sum_control);
        ros::spinOnce();

        loop_rate.sleep();
        ++count;
    }
    return 0;
}
