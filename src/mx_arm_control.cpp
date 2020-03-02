#include "ros/ros.h"
#include <dynamixel_workbench_msgs/DynamixelCommand.h>
#include <cstdlib>
#include <math.h>
#include <chrono>
#include <vector>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JointState.h>
#include <control_msgs/FollowJointTrajectoryGoal.h>
#include <thread>
#include <utility>

#define PI 3.141592654
#define Number_Motors 3
int mx_offset = 3; //first mx motor is in mx_offset joint
using namespace std;

vector<int> motor_ids{44, 55, 66};
vector<pair<int, int>> min_max_values_{make_pair(0, 4095), make_pair(780, 3450), make_pair(0, 4095)}; //joint 4 make_pair(0, 4095)
vector<double> resolutions_{4096.0 / 360.0, 4096.0 / 360.0, 4096.0 / 360.0};                          //4th joint 4096 / 360
vector<double> offsets_{0.0, -0.0, 0.0};
vector<int> motor_velocities_{35, 15, 35};

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
sensor_msgs::JointState Joints_State;
void JointsState_Handler(const sensor_msgs::JointState &joints_state)
{
    Joints_State = joints_state;
}

int Convert_Angle_to_Value(double angle, int motor_index)
{
    angle += 180;
    int value = round(angle * resolutions_[motor_index]);
    if (value > min_max_values_[motor_index].second)
        value = min_max_values_[motor_index].second;
    if (value < min_max_values_[motor_index].first)
        value = min_max_values_[motor_index].first;

    return value;
}
int Convert_Radian_to_Value(double radian, int motor_index)
{
    //std::cout << "radian " << radian << std::endl;
    radian += offsets_[motor_index];
    //std::cout << "radian later " << radian << std::endl;
    double angle = radian * 57.2957779513;
    //std::cout << "angle " << angle << std::endl;
    angle += 180; //as zero degrees is at 2048, offset in values

    //std::cout << "angle later " << angle << std::endl;
    int value = round(angle * resolutions_[motor_index]);
    if (value > min_max_values_[motor_index].second)
        value = min_max_values_[motor_index].second;
    if (value < min_max_values_[motor_index].first)
        value = min_max_values_[motor_index].first;

    // std::cout << "value " << value << std::endl;

    return value;
}
int main(int argc, char **argv)
{

    ros::init(argc, argv, "mx_arm_controller");

    ros::NodeHandle ros_node_handler;
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    bool service_state = ros::service::exists("/dynamixel_workbench_mx/dynamixel_command", true);
    cout << "service state " << service_state << endl;
    ros::ServiceClient client = ros_node_handler.serviceClient<dynamixel_workbench_msgs::DynamixelCommand>("/dynamixel_workbench_mx/dynamixel_command");
    ros::Subscriber sub_Trajectory = ros_node_handler.subscribe("/robot1/arm_general/goal_command", 2, Trajectory_Handler);
    ros::Subscriber sub_joystick = ros_node_handler.subscribe("/joy", 1, Joy_Handler); //Joystick

    ros::Subscriber sub_joint_state = ros_node_handler.subscribe("/dynamixel_workbench_mx/joint_states", 1, JointsState_Handler); //Joint State

    ros::Rate loop_rate(45);
    dynamixel_workbench_msgs::DynamixelCommand command_Position;
    command_Position.request.command = "";
    command_Position.request.addr_name = string("Goal_Position");

    dynamixel_workbench_msgs::DynamixelCommand command_Torque;
    command_Torque.request.addr_name = string("Torque_Enable");
    bool torque_enabled = false;
    int count = 0;
    int odd = 0;

    dynamixel_workbench_msgs::DynamixelCommand command_Properties;
    command_Properties.request.command = "";
    for (int j = 0; j < Number_Motors; j++)
    {
        command_Properties.request.addr_name = string("Torque_Enable");
        command_Properties.request.id = motor_ids[j];
        command_Properties.request.value = 0;
        if (j != 1)
        {
            client.call(command_Properties);
        }
        command_Properties.request.addr_name = string("Drive_Mode");
        command_Properties.request.id = motor_ids[j];
        command_Properties.request.value = 3;
        client.call(command_Properties);

        command_Properties.request.addr_name = string("Profile_Velocity");
        command_Properties.request.value = motor_velocities_[j]; //17*0.229 = 3.893 RPM  MX64
        client.call(command_Properties);
    }
    vector<double> old_motor_positions_ = offsets_;
    while (ros::ok())
    {
        bool joystick_state = false;

        if (!new_joy_message_received || !new_trajectory_received)
        {
            joystick_state = false; //just to bypass the first iterations when joy and traj is not available
        }
        else if (joystick_msg.axes[5] != -1.0)
        {
            joystick_state = false;
        }
        else
        {
            //cout << "jy " << joystick_msg.axes[5] << endl;
            joystick_state = true;
        }
        if (!joystick_state)
        {
            for (int j = 0; j < Number_Motors; j++)
            {
                command_Torque.request.addr_name = string("Torque_Enable");
                command_Torque.request.id = motor_ids[j];
                command_Torque.request.value = 0;
                // if (j != 1)
                //  client.call(command_Torque);  //NOT DISABLING FOR NOW
            }
            torque_enabled = false;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
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

                    command_Properties.request.id = motor_ids[j];
                    command_Properties.request.addr_name = string("Profile_Velocity");
                    command_Properties.request.value = motor_velocities_[j]; //17*0.229 = 3.893 RPM  MX64
                    client.call(command_Properties);
                }
                torque_enabled = true;
            }
        }

        for (int i = 0; i < Number_Motors; i++)
        {
            double request_value = Convert_Radian_to_Value(trajectory_goal.trajectory.points[0].positions[i + mx_offset], i);
            command_Position.request.id = motor_ids[i];
            command_Position.request.value = request_value;
            //if (old_motor_positions_[i] != request_value)
            //{
            client.call(command_Position);
            //}
            old_motor_positions_[i] = request_value;
            // std::cout << "Joint" << motor_ids[i] << ", request: " << trajectory_goal.trajectory.points[0].positions[i + mx_offset] << ", transformed value: " << request_value << std::endl;
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