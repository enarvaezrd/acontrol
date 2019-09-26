#include "ros/ros.h"
#include <dynamixel_workbench_msgs/DynamixelCommand.h>
#include <cstdlib>
#include <math.h>
#include <chrono>
#include <vector>
#include <sensor_msgs/Joy.h>
#include <control_msgs/FollowJointTrajectoryGoal.h>
#include <thread>
#include <utility>

#define PI 3.141592654
#define Number_Motors 2

using namespace std;

vector<int> motor_ids{1, 2};
vector<pair<int, int>> min_max_values_{make_pair(-303750, 303750), make_pair(-175000, 175000)};
vector<double> resolutions_{607500 / 360, 501923 / 360};
vector<double> offsets_{-PI/2 , 0.0};

bool new_joy_message_received = false;
sensor_msgs::Joy joystick_msg;
void Joy_Handler(const sensor_msgs::Joy &joystick)
{
    joystick_msg = joystick;
    new_joy_message_received = true;
}

control_msgs::FollowJointTrajectoryGoal trajectory_goal;
bool new_trajectory_received = false;
void Trajectory_Handler(const control_msgs::FollowJointTrajectoryGoal &traj_goal)
{
    trajectory_goal = traj_goal;
    new_trajectory_received = true;
}
int Convert_Angle_to_Value(double angle, int motor_index)
{
    if (motor_index == 0)
    {
        angle = (-1) * angle;
    }
    int value = round(angle * resolutions_[motor_index]);
    if (value > min_max_values_[motor_index].second)
        value = min_max_values_[motor_index].second;
    if (value < min_max_values_[motor_index].first)
        value = min_max_values_[motor_index].first;

    return value;
}
int Convert_Radian_to_Value(double radian, int motor_index)
{
    radian +=offsets_[motor_index]; 
    double angle = radian * 57.2957779513;
    if (motor_index == 0)
    {
        angle = (-1) * angle;
    }
    int value = round(angle * resolutions_[motor_index]);
    if (value > min_max_values_[motor_index].second)
        value = min_max_values_[motor_index].second;
    if (value < min_max_values_[motor_index].first)
        value = min_max_values_[motor_index].first;

    return value;
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "pro_arm_controller");

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    ros::NodeHandle ros_node_handler;
    bool service_state = ros::service::exists("/dynamixel_workbench_pro/dynamixel_command", true);
    cout << "service state " << service_state << endl;
    ros::ServiceClient client = ros_node_handler.serviceClient<dynamixel_workbench_msgs::DynamixelCommand>("/dynamixel_workbench_pro/dynamixel_command");
    ros::Subscriber sub_Trajectory = ros_node_handler.subscribe("/robot2/arm_general/goal_command", 2, Trajectory_Handler);
    ros::Subscriber sub_joystick = ros_node_handler.subscribe("/joy", 1, Joy_Handler); //Joystick

    ros::Rate loop_rate(3);
    dynamixel_workbench_msgs::DynamixelCommand command_Position;
    command_Position.request.command = "";
    command_Position.request.addr_name = string("Goal_Position");

    dynamixel_workbench_msgs::DynamixelCommand command_Torque;
    command_Torque.request.command = "";
    command_Torque.request.addr_name = string("Torque_Enable");
    bool torque_enabled = false;
    int count = 0;
    int odd = 0;
    while (ros::ok())
    {
        bool joystick_state = false;

        if (!new_joy_message_received || !new_trajectory_received)
        {

            joystick_state = false; //just to bypass the first iterations when joy is not available
        }
        else if (joystick_msg.axes[5] != -1.0)
        {

            joystick_state = false;
        }
        else
        {
            cout << "jy " << joystick_msg.axes[5] << endl;
            joystick_state = true;
        }
        if (!joystick_state)
        {
            for (int j = 0; j < Number_Motors; j++)
            {
                command_Torque.request.addr_name = string("Torque_Enable");
                command_Torque.request.id = motor_ids[j];
                command_Torque.request.value = 0;
                client.call(command_Torque);
            }
            torque_enabled = false;
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            ros::spinOnce();
            loop_rate.sleep();
            continue;
        }
        else
        {
            if (!torque_enabled)
            {
                for (int j = 0; j < Number_Motors; j++)
                {
                    command_Torque.request.id = motor_ids[j];
                    command_Torque.request.value = 1;
                    client.call(command_Torque);
                }
                torque_enabled = true;
            }
        }

        for (int i = 0; i < Number_Motors; i++)
        {
            double request_value = Convert_Radian_to_Value(trajectory_goal.trajectory.points[0].positions[i], i);
            command_Position.request.id = motor_ids[i];
            command_Position.request.value = request_value;
            client.call(command_Position);
        }

        loop_rate.sleep();
        ros::spinOnce();
    }
    for (int j = 0; j < Number_Motors; j++)
    {
        command_Torque.request.id = motor_ids[j];
        command_Torque.request.value = 0;
        client.call(command_Torque);
    }
    return 0;
}