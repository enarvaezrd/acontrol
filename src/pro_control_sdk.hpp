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

#include "dynamixel_sdk/dynamixel_sdk.h" // Uses Dynamixel SDK library
using namespace std;

struct motor_params
{
    int ID;
    pair<int, int> motor_limits;
    double resolution;
    double offset = 0.0;
    double protocol_version = 2.0;
    int moving_threshold = 20;
    int torque_enable_addr = 562;    //pro H54 H42 , 512 for PRO H42P
    int led_red_addr = 563;          //513
    int goal_position_addr = 596;    //564
    int present_position_addr = 611; //580
    int velocity_limit_addr = 32;    //44
    int goal_position;
    int32_t current_position = 0;
    double current_angle = 0.0;
    int velocity_limit = 10;
    int reverse = 1; //reverse movement
};
class Dynamixel_SDK_Handler
{
public:
    int baud_rate;
    double packet_timeout;
    dynamixel::PortHandler *portHandler;
    dynamixel::PacketHandler *packetHandler;
    dynamixel::GroupBulkWrite groupBulkWrite;
    dynamixel::GroupBulkRead groupBulkRead;
    vector<motor_params> motor_parameters;
    int number_motors;
    int minimum_value_for_movement;

    bool new_joy_message_received = false;
    sensor_msgs::Joy joystick_msg;
    bool motors_stoped;
    bool death_man_state;
    ros::Publisher joint_states_pub;
    ros::NodeHandle nh;

    Dynamixel_SDK_Handler(const char *dev_name, double protocol_version, int baud_rate_, double packet_timeout_, int min_mov_value_)
        : baud_rate(baud_rate_), packet_timeout(packet_timeout_), portHandler(dynamixel::PortHandler::getPortHandler(dev_name)),
          packetHandler(dynamixel::PacketHandler::getPacketHandler(protocol_version)),
          groupBulkWrite(dynamixel::GroupBulkWrite(portHandler, packetHandler)),
          groupBulkRead(dynamixel::GroupBulkRead(portHandler, packetHandler)), number_motors(0),
          minimum_value_for_movement(min_mov_value_), new_joy_message_received(false),
          death_man_state(false), motors_stoped(true)
    {
        joint_states_pub = nh.advertise<sensor_msgs::JointState>("/dynamixel_ed_pro_control/joint_states", 1);
    }

    void Joy_Handler(const sensor_msgs::Joy &joystick)
    {
        joystick_msg = joystick;
        new_joy_message_received = true;

        if (joystick_msg.axes[5] != -1.0)
            death_man_state = false;
        else
            death_man_state = true;
        return;
    }

    bool Open_Port()
    {
        // Open port
        if (portHandler->openPort())
        {
            printf("Succeeded to open the port!\n");
        }
        else
        {
            printf("Failed to open the port!\n");
            return false;
        }
        portHandler->setPacketTimeout(packet_timeout);
        // Set port baudrate
        if (portHandler->setBaudRate(baud_rate))
        {
            printf("Succeeded to change the baudrate!\n");
        }
        else
        {
            printf("Failed to change the baudrate!\n");
            return false;
        }
        return true;
    }
    void Initialize_Motor_Parameters(motor_params parameters)
    {
        motor_parameters.push_back(parameters);
        number_motors = motor_parameters.size();
    }
    bool Write_Velocity_Limits()
    {
        Disable_Torques();
        uint8_t dxl_error = 0;
        int dxl_comm_result = COMM_TX_FAIL;

        bool dxl_addparam_result = false;
        for (auto motor : motor_parameters)
        {
            dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, motor.ID, motor.velocity_limit_addr, motor.velocity_limit, &dxl_error);
            if (dxl_comm_result != COMM_SUCCESS)
            {
                printf("[ID: %03d] No success writing Vel limit, %s\n", motor.ID, packetHandler->getTxRxResult(dxl_comm_result));
                return false;
            }
            else if (dxl_error != 0)
            {
                printf("[ID: %03d] Error writing Vel limit, %s\n", motor.ID, packetHandler->getRxPacketError(dxl_error));
                return false;
            }
            else
            {
                printf("DXL#%d velocity limit has been successfully written \n", motor.ID);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        return true;
    }
    bool Enable_Torques()
    {
        uint8_t dxl_error = 0;
        int dxl_comm_result = COMM_TX_FAIL;

        bool dxl_addparam_result = false;
        for (auto motor : motor_parameters)
        {
            dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, motor.ID, motor.torque_enable_addr, 1, &dxl_error);
            if (dxl_comm_result != COMM_SUCCESS)
            {
                printf("[ID: %03d] No success enabling, %s\n", motor.ID, packetHandler->getTxRxResult(dxl_comm_result));
                return false;
            }
            else if (dxl_error != 0)
            {
                printf("[ID: %03d] Error enabling, %s\n", motor.ID, packetHandler->getRxPacketError(dxl_error));
                return false;
            }
            else
            {
                printf("DXL#%d has been successfully enabled \n", motor.ID);
            }
        }
        return true;
    }
    bool Prepare_Bulk_Reader()
    {
        bool dxl_addparam_result = false;
        for (auto motor : motor_parameters)
        {
            dxl_addparam_result = groupBulkRead.addParam(motor.ID, motor.present_position_addr, 4);
            if (dxl_addparam_result != true)
            {
                fprintf(stderr, "[ID:%03d] Preparing Reader, grouBulkRead addparam failed", motor.ID);
                return false;
            }
        }
        return true;
    }
    void Publish_Joints()
    {
        if (number_motors > 0)
        {
            sensor_msgs::JointState joint_states;
            for (int i = 0; i < number_motors; i++)
            {
                joint_states.position.push_back(motor_parameters[i].current_angle);
                joint_states.name.push_back(to_string(motor_parameters[i].ID));
            }
            joint_states_pub.publish(joint_states);
        }
        return;
    }
    bool Bulk_Write_Position_Goals(vector<double> position_goals_rad)
    {
        if (position_goals_rad.size() != number_motors)
        {
            cerr << "Write goals. Number of motors different from goals number\n";
            return false;
        }
        if (!death_man_state)
        {
            return false;
        }
        bool dxl_addparam_result = false;
        bool dxl_getdata_result = false;
        int dxl_comm_result = COMM_TX_FAIL;
        for (int i = 0; i < number_motors; i++)
        {
            int goal = Convert_Radian_to_Value(position_goals_rad[i], i);

            uint8_t param_goal_position[4];
            param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(goal));
            param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(goal));
            param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(goal));
            param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(goal));

            dxl_addparam_result = groupBulkWrite.addParam(motor_parameters[i].ID, motor_parameters[i].goal_position_addr, 4, param_goal_position);
            if (dxl_addparam_result != true)
            {
                fprintf(stderr, "[ID:%03d] groupBulkWrite addparam failed", motor_parameters[i].ID);
                return false;
            }
            motor_parameters[i].goal_position = goal;
        }
        dxl_comm_result = groupBulkWrite.txPacket();
        if (dxl_comm_result != COMM_SUCCESS)
            printf("No success reading,  %s\n", packetHandler->getTxRxResult(dxl_comm_result));
        groupBulkWrite.clearParam();
        motors_stoped = false;
        bool all_joints_completion;
        do
        {
            int reading_count = 0;
            do
            {
                dxl_comm_result = groupBulkRead.txRxPacket();
                if (dxl_comm_result != COMM_SUCCESS)
                {
                    //printf("Error write position goals txRxPacket: %s, retrying! joy state: %d \n", packetHandler->getTxRxResult(dxl_comm_result),death_man_state);
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                }
                reading_count++;
            } while (dxl_comm_result != COMM_SUCCESS && death_man_state && reading_count > 30); //stop at about 200ms
            if (!death_man_state)
            {
                break;
            }
            all_joints_completion = true;
            vector<double> position_differences;
            for (int i = 0; i < number_motors; i++)
            {
                // Check if groupbulkread data of Dynamixel#1 is available
                dxl_getdata_result = groupBulkRead.isAvailable(motor_parameters[i].ID, motor_parameters[i].present_position_addr, 4);
                if (dxl_getdata_result != true)
                {
                    //fprintf(stderr, "[ID:%03d] groupBulkRead getdata failed\n", motor_parameters[i].ID);
                    return false;
                }
                // Get present position value
                motor_parameters[i].current_position = groupBulkRead.getData(motor_parameters[i].ID, motor_parameters[i].present_position_addr, 4);
                if (abs(motor_parameters[i].current_position - motor_parameters[i].goal_position) > minimum_value_for_movement)
                {
                    all_joints_completion = false;
                }
                motor_parameters[i].current_angle = motor_parameters[i].reverse * Convert_Value_to_Radian(motor_parameters[i].current_position, i);

                //  printf("[ID:%03d] Goal: %d,  Current Position : %d, Current angle: %f\n", motor_parameters[i].ID, motor_parameters[i].goal_position, motor_parameters[i].current_position, motor_parameters[i].current_angle);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            Publish_Joints();
        } while (!all_joints_completion && death_man_state);
        if (death_man_state)
            Stop_Motors();
        return true;
    }
    void Stop_Motors()
    { //stop movements by sending current joints  position
        bool dxl_addparam_result = false;
        bool dxl_getdata_result = false;
        int dxl_comm_result = COMM_TX_FAIL;
        do
        {
            dxl_comm_result = groupBulkRead.txRxPacket();
            if (dxl_comm_result != COMM_SUCCESS)
            {
                //printf("Error Stopping: %s, retrying!\n", packetHandler->getTxRxResult(dxl_comm_result));
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        } while (dxl_comm_result != COMM_SUCCESS);

        for (int i = 0; i < number_motors; i++)
        {

            dxl_getdata_result = groupBulkRead.isAvailable(motor_parameters[i].ID, motor_parameters[i].present_position_addr, 4);
            if (dxl_getdata_result != true)
            {
                //fprintf(stderr, "[ID:%03d] groupBulkRead getdata failed\n", motor_parameters[i].ID);
                return;
            }
            motor_parameters[i].current_position = groupBulkRead.getData(motor_parameters[i].ID, motor_parameters[i].present_position_addr, 4);
            motor_parameters[i].current_angle = motor_parameters[i].reverse * Convert_Value_to_Radian(motor_parameters[i].current_position, i);
            //printf("[ID:%03d] Goal: %d,  Current Position : %d \n", motor_parameters[i].ID, motor_parameters[i].goal_position, motor_parameters[i].current_position);
        }
        if (!motors_stoped)
        {

            for (int i = 0; i < number_motors; i++)
            {
                int goal = motor_parameters[i].current_position;

                uint8_t param_goal_position[4];
                param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(goal));
                param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(goal));
                param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(goal));
                param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(goal));

                dxl_addparam_result = groupBulkWrite.addParam(motor_parameters[i].ID, motor_parameters[i].goal_position_addr, 4, param_goal_position);
                if (dxl_addparam_result != true)
                {
                    fprintf(stderr, "[ID:%03d] groupBulkWrite addparam failed", motor_parameters[i].ID);
                    return;
                }
            }

            dxl_comm_result = groupBulkWrite.txPacket();
            if (dxl_comm_result != COMM_SUCCESS)
                printf("No success reading,  %s\n", packetHandler->getTxRxResult(dxl_comm_result));
            groupBulkWrite.clearParam();
            motors_stoped = true;
        }
        Publish_Joints();
        return;
    }
    bool Disable_Torques()
    {
        uint8_t dxl_error = 0;
        int dxl_comm_result = COMM_TX_FAIL;

        bool dxl_addparam_result = false;
        for (auto motor : motor_parameters)
        {
            dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, motor.ID, motor.torque_enable_addr, 0, &dxl_error);
            if (dxl_comm_result != COMM_SUCCESS)
            {
                printf("[ID: %03d] No success disabling, %s\n", motor.ID, packetHandler->getTxRxResult(dxl_comm_result));
                return false;
            }
            else if (dxl_error != 0)
            {
                printf("[ID: %03d] Error disabling, %s\n", motor.ID, packetHandler->getRxPacketError(dxl_error));
                return false;
            }
            else
            {
             //   printf("DXL#%d has been successfully disabled \n", motor.ID);
            }
        }
        return true;
    }
    bool Disable_Individual_Torque(int index)
    {
        uint8_t dxl_error = 0;
        int dxl_comm_result = COMM_TX_FAIL;

        bool dxl_addparam_result = false;
        auto motor = motor_parameters[index];

        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, motor.ID, motor.torque_enable_addr, 0, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            printf("[ID: %03d] No success disabling, %s\n", motor.ID, packetHandler->getTxRxResult(dxl_comm_result));
            return false;
        }
        else if (dxl_error != 0)
        {
            printf("[ID: %03d] Error disabling, %s\n", motor.ID, packetHandler->getRxPacketError(dxl_error));
            return false;
        }
        else
        {
            //printf("DXL#%d has been successfully disabled \n", motor.ID);
        }
        return true;
    }
    void Close_Port()
    {
        portHandler->closePort();
        return;
    }
    int Convert_Radian_to_Value(double radian, int motor_index)
    {
        radian += motor_parameters[motor_index].offset;
        double angle = radian * 57.2957779513;
        int value = Convert_Angle_to_Value(angle, motor_index);
        return value;
    }
    int Convert_Angle_to_Value(double angle, int motor_index)
    {
        angle *= motor_parameters[motor_index].reverse;
        int value = round(angle * motor_parameters[motor_index].resolution);
        if (value > motor_parameters[motor_index].motor_limits.second)
            value = motor_parameters[motor_index].motor_limits.second;
        if (value < motor_parameters[motor_index].motor_limits.first)
            value = motor_parameters[motor_index].motor_limits.first;
        return value;
    }

    double Convert_Value_to_Radian(int value, int motor_index)
    {
        double angle = (double)(value) / motor_parameters[motor_index].resolution;
        //cout << "[" << motor_index << "], angle: " << angle << endl;
        double radian = angle / 57.2957779513;
        return radian;
    }
};