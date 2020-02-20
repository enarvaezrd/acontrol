
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

// Data Byte Length
#define LEN_PRO_LED_RED 1
#define LEN_PRO_GOAL_POSITION 4
#define LEN_PRO_PRESENT_POSITION 4

using namespace std;

struct motor_params
{
    uint8_t ID;
    pair<int, int> motor_limits;
    double resolution;
    double offset = 0.0;
    double protocol_version = 2.0;
    int moving_threshold = 20;
    int torque_enable_addr = 562;         //pro H54 H42 , 512 for PRO H42P
    int led_red_addr = 563;               //513
    uint16_t goal_position_addr = 596;    //564
    uint16_t present_position_addr = 611; //580
    uint16_t velocity_limit_addr = 32;    //44
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
    //dynamixel::PacketHandler *packetHandlerVel;
    dynamixel::GroupBulkWrite groupBulkWrite;
    //dynamixel::GroupBulkWrite groupBulkWriteVel;
    dynamixel::GroupBulkRead groupBulkRead;
    // dynamixel::GroupBulkRead groupBulkRead_VelLimits;
    vector<motor_params> motor_parameters;
    int number_motors;
    int minimum_value_for_movement;

    bool new_joy_message_received = false;
    sensor_msgs::Joy joystick_msg;
    bool motors_stoped;
    bool death_man_state;
    ros::Publisher joint_states_pub;
    ros::NodeHandle nh;

    Dynamixel_SDK_Handler(const char *dev_name, float protocol_version, int baud_rate_, double packet_timeout_, int min_mov_value_)
        : baud_rate(baud_rate_), packet_timeout(packet_timeout_), portHandler(dynamixel::PortHandler::getPortHandler(dev_name)),
          packetHandler(dynamixel::PacketHandler::getPacketHandler(protocol_version)),
          groupBulkWrite(dynamixel::GroupBulkWrite(portHandler, packetHandler)),
          groupBulkRead(dynamixel::GroupBulkRead(portHandler, packetHandler)),
          // groupBulkRead_VelLimits(dynamixel::GroupBulkRead(portHandler, packetHandlerVel)),
          number_motors(0), minimum_value_for_movement(min_mov_value_), new_joy_message_received(false),
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
            printf("[INFO]: Pro_control_SDK, Succeeded to open the port!\n");
        }
        else
        {
            printf("[ERROR]: Pro_control_SDK, Failed to open the port!\n");
            return false;
        }
        portHandler->setPacketTimeout(packet_timeout);
        // Set port baudrate
        if (portHandler->setBaudRate(baud_rate))
        {
            printf("[INFO]: Pro_control_SDK, Succeeded to change the baudrate!\n");
        }
        else
        {
            printf("[ERROR]: Pro_control_SDK, Failed to change the baudrate!\n");
            return false;
        }
        return true;
    }
    void Initialize_Motor_Parameters(motor_params parameters)
    {
        motor_parameters.push_back(parameters);
        number_motors = motor_parameters.size();
        printf("Motor parameter loaded: ID: %d\n", parameters.ID);
    }
    bool Write_Velocity_Limits()
    {
        Disable_Torques_Bulk();
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
    /* bool Write_Velocity_Limits_Bulk()
    {
        Disable_Torques_Bulk();
        bool dxl_addparam_result = false;
        bool dxl_getdata_result = false;
        int dxl_comm_result = COMM_TX_FAIL;
        int goal = 0;

        for (auto motor : motor_parameters)
        {
            uint8_t param_velocity_value[4];
            param_velocity_value[0] = DXL_LOBYTE(DXL_LOWORD(motor.velocity_limit));
            param_velocity_value[1] = DXL_LOBYTE(DXL_LOWORD(motor.velocity_limit));
            param_velocity_value[2] = DXL_LOBYTE(DXL_LOWORD(motor.velocity_limit));
            param_velocity_value[3] = DXL_LOBYTE(DXL_LOWORD(motor.velocity_limit));
            dxl_addparam_result = groupBulkWrite.addParam(motor.ID, motor.velocity_limit_addr, 4, param_velocity_value);
            if (dxl_addparam_result != true)
            {
                fprintf(stderr, "[ID:%03d] Bulk Velocity Writter: groupBulkWriteVel addparam failed", motor.ID);
                return false;
            }
        }
        dxl_comm_result = groupBulkWrite.txPacket();
        groupBulkWriteVel.clearParam();
        if (dxl_comm_result != COMM_SUCCESS)
        {
            ROS_ERROR("Write VELOCITY Limits: No success writting,  %s\n", packetHandler->getTxRxResult(dxl_comm_result));
            return false;
        }
        else
        {
            fprintf(stderr, " Bulk Velocity Writter: success writting! %s\n", packetHandler->getTxRxResult(dxl_comm_result));
        }
        return true;
    }*/
    /* bool Check_Velocity_Limits()
    {
        bool dxl_addparam_result = false;
        bool dxl_getdata_result = false;
        int dxl_comm_result = COMM_TX_FAIL;
        do
        {
            dxl_comm_result = groupBulkRead_VelLimits.txRxPacket();
            if (dxl_comm_result != COMM_SUCCESS)
            {
                printf("Error Reading Velocities: %s, retrying!\n", packetHandlerVel->getTxRxResult(dxl_comm_result));
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        } while (dxl_comm_result != COMM_SUCCESS && ros::ok());

        for (auto motor : motor_parameters)
        {
            dxl_getdata_result = groupBulkRead_VelLimits.isAvailable(motor.ID, motor.velocity_limit_addr, 4);
            if (dxl_getdata_result != true)
            {
                fprintf(stderr, "[ID:%03d] groupBulkRead_VelLimits getdata Velocity failed, not available\n", motor.ID);
                return false;
            }
            int motor_vel = groupBulkRead_VelLimits.getData(motor.ID, motor.present_position_addr, 4);
            if (motor_vel != motor.velocity_limit)
            {
                ROS_ERROR("[ID: %d] Motor velocity (%d) not equal to requirement (%d)\n", motor.ID, motor_vel, motor.velocity_limit);
                return false;
            }
            else
            {
                ROS_INFO("[ID: %d] Motor velocity (%d) equal to requirement (%d)\n", motor.ID, motor_vel, motor.velocity_limit);
            }
        }
        return true;
    }*/
    bool Enable_Torques()
    {
        uint8_t dxl_error = 0;
        int dxl_comm_result = COMM_TX_FAIL;

        bool dxl_addparam_result = false;
        for (auto motor : motor_parameters)
        {
            int enable_cnt = 0;
            do
            {
                enable_cnt++;
                dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, motor.ID, motor.torque_enable_addr, 1, &dxl_error);
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            } while (dxl_comm_result != COMM_SUCCESS && ros::ok() && enable_cnt < 50);

            if (dxl_comm_result != COMM_SUCCESS)
            {
#ifdef DEBUG
                printf("[ID: %03d] No success enabling, %s\n", motor.ID, packetHandler->getTxRxResult(dxl_comm_result));
#endif
                return false;
            }
            else if (dxl_error != 0)
            {
#ifdef DEBUG
                printf("[ID: %03d] Error enabling, %s\n", motor.ID, packetHandler->getRxPacketError(dxl_error));
#endif
                return false;
            }
            else
            {
                // printf("DXL#%d has been successfully enabled \n", motor.ID);
            }
        }
        return true;
    }
    bool Enable_Torques_Bulk()
    {
        bool dxl_addparam_result = false;
        int dxl_comm_result = COMM_TX_FAIL;
        int goal = 1;
        uint8_t param_enable_value[1];
        param_enable_value[0] = DXL_LOBYTE(DXL_LOWORD(goal));
        for (auto motor : motor_parameters)
        {

            dxl_addparam_result = groupBulkWrite.addParam(motor.ID, motor.torque_enable_addr, 1, param_enable_value);
            if (dxl_addparam_result != true)
            {
                // fprintf(stderr, "[ID:%03d] Enable Motors Torque: groupBulkWrite addparam failed", motor.ID);
                return false;
            }
        }
        dxl_comm_result = groupBulkWrite.txPacket();
        groupBulkWrite.clearParam();
        if (dxl_comm_result != COMM_SUCCESS)
        {
#ifdef DEBUG
            ROS_ERROR("Write Enabling Value: No success reading,  %s\n", packetHandler->getTxRxResult(dxl_comm_result));
#endif
            return false;
        }
        return true;
    }
    bool Prepare_Bulk_Reader()
    {
        ROS_INFO("PROTOCOL VERSION %d", (int)(groupBulkRead.getPacketHandler()->getProtocolVersion()));
        //groupBulkRead.clearParam();
        bool dxl_addparam_result = false;
        for (auto motor : motor_parameters)
        {

            dxl_addparam_result = groupBulkRead.addParam(motor.ID, motor.present_position_addr, LEN_PRO_PRESENT_POSITION);
            ROS_INFO("PREPARING READER JOINTS ID: %d, Addr: %d", motor.ID, motor.present_position_addr);
            if (dxl_addparam_result != COMM_SUCCESS)
            {
                ROS_ERROR("[ID:%03d] Preparing Reader, grouBulkRead addparam failed", motor.ID);
                return false;
            }
        }
        /*groupBulkRead_VelLimits.clearParam();
        for (auto motor : motor_parameters)
        {
            dxl_addparam_result = groupBulkRead_VelLimits.addParam(motor.ID, motor.velocity_limit_addr, 4);
            if (dxl_addparam_result != true)
            {
                ROS_ERROR("[ID:%03d] Preparing Reader, grouBulkRead_VELLIMITS addparam failed", motor.ID);
                return false;
            }
        }*/

        return true;
    }
    void Read_Individual_and_Publish_Joints()
    {

        for (int i = 0; i < number_motors; i++)
        {
            int32_t dxl_present_position = 0;
            uint8_t dxl_error = 0;
            int dxl_comm_result = COMM_TX_FAIL;
            int enable_cnt = 0;
            do
            {
                enable_cnt++;
                dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, motor_parameters[i].ID, motor_parameters[i].present_position_addr, (uint32_t *)&dxl_present_position, &dxl_error);
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            } while (dxl_comm_result != COMM_SUCCESS && ros::ok() && enable_cnt < 5);

            if (dxl_comm_result != COMM_SUCCESS)
            {
#ifdef DEBUG
                ROS_INFO("[INDIVIDUAL %d] no success txRxPacket: %s \n", i, packetHandler->getTxRxResult(dxl_comm_result));
#endif
            }
            else if (dxl_error != 0)
            {
#ifdef DEBUG
                ROS_INFO("[INDIVIDUAL %d] ERROR txRxPacket: %s \n", i, packetHandler->getRxPacketError(dxl_error));
#endif
            }
            int *p2;
            //memcpy(dxl_present_position, p1, 100 * sizeof(int));
            motor_parameters[i].current_position = dxl_present_position;

            motor_parameters[i].current_angle = motor_parameters[i].reverse * Convert_Value_to_Radian(motor_parameters[i].current_position, i);
            #ifdef DEBUG
            printf("[ID:%03d]  PresPos:%03d\n", motor_parameters[i].ID, dxl_present_position);
            #endif
        }
        Publish_Joints();
    }
    void Read_and_Publish_Joints()
    {
        int joints_intents_count = 0;
        int updated_joints = 0;

        int reading_count = 0;
        bool dxl_comm_result = false;
        bool prepare_param_flag = false;
        //while(!prepare_param_flag)
        //{
        //     prepare_param_flag=Prepare_Bulk_Reader();
        // }
        do
        {
            dxl_comm_result = groupBulkRead.txRxPacket();
            if (dxl_comm_result != COMM_SUCCESS)
            {
                //printf("Error write position goals txRxPacket: %s, retrying! joy state: %d \n", packetHandler->getTxRxResult(dxl_comm_result), death_man_state);
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                reading_count++;
            }

        } while (dxl_comm_result != COMM_SUCCESS && reading_count < 10 && ros::ok());

        if (dxl_comm_result != COMM_SUCCESS)
        {
            #ifdef DEBUG
            ROS_INFO("CANT READ FROM JOINTS");
            #endif
            //return;
        }
        else
        {
            #ifdef DEBUG
            ROS_INFO("READ SUCCESS FROM JOINTS %d", number_motors);
            #endif
        };
        
        for (int i = 0; i < number_motors; i++)
        {
            // Check if groupbulkread data of Dynamixel#1 is available
            bool dxl_getdata_result = false;
            int get_data_cnt = 0;
           // ROS_INFO("READING JOINTS ID: %d, Addr: %d", motor_parameters[i].ID, motor_parameters[i].present_position_addr);
            do
            {
                dxl_getdata_result = groupBulkRead.isAvailable(motor_parameters[i].ID, motor_parameters[i].present_position_addr, LEN_PRO_PRESENT_POSITION);
                std::this_thread::sleep_for(std::chrono::milliseconds(6));
                get_data_cnt++;
            } while (!dxl_getdata_result && ros::ok() && get_data_cnt < 5);

            if (dxl_getdata_result != true)
            {
                continue;
            }
           // ROS_INFO("READ SUCCESS JOINTS ID: %d, Addr: %d", motor_parameters[i].ID, motor_parameters[i].present_position_addr);
            // Get present position value
            motor_parameters[i].current_position = groupBulkRead.getData(motor_parameters[i].ID, motor_parameters[i].present_position_addr, LEN_PRO_PRESENT_POSITION);

            motor_parameters[i].current_angle = motor_parameters[i].reverse * Convert_Value_to_Radian(motor_parameters[i].current_position, i);
            updated_joints++;
        }
        // groupBulkRead.clearParam();
       // ROS_INFO("UPDATED JOINTS %d", updated_joints);
        Publish_Joints();
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
            cerr << "Pro_Arm Control, Write goals: Number of motors different from goals number\n";
            return false;
        }
        if (!death_man_state)
        {
            return true;
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

            dxl_addparam_result = groupBulkWrite.addParam(motor_parameters[i].ID, motor_parameters[i].goal_position_addr, LEN_PRO_GOAL_POSITION, param_goal_position);
            if (dxl_addparam_result != true)
            {
                fprintf(stderr, "[ID:%03d] groupBulkWrite addparam failed", motor_parameters[i].ID);
                return false;
            }
            motor_parameters[i].goal_position = goal;
        }
        dxl_comm_result = groupBulkWrite.txPacket();
        groupBulkWrite.clearParam();
        if (dxl_comm_result != COMM_SUCCESS)
        {
            // printf("Write Positions: No success reading,  %s\n", packetHandler->getTxRxResult(dxl_comm_result));
            return false;
        }
        // Prepare_Bulk_Reader();
        motors_stoped = false;
        bool all_joints_completion;
        int joints_intents_count = 0;
        do
        {
            int reading_count = 0;
            do
            {
                dxl_comm_result = groupBulkRead.txRxPacket();
                if (dxl_comm_result != COMM_SUCCESS)
                {
                    //printf("Error write position goals txRxPacket: %s, retrying! joy state: %d \n", packetHandler->getTxRxResult(dxl_comm_result), death_man_state);
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                    reading_count++;
                }

            } while (dxl_comm_result != COMM_SUCCESS && death_man_state && reading_count <2 && ros::ok());

            if (!dxl_comm_result)
            {
                return false;
            }
            if (!death_man_state)
            {
                return true;
            }
            all_joints_completion = true;
            vector<double> position_differences;
            int updated_joints = 0;
            for (int i = 0; i < number_motors; i++)
            {
                // Check if groupbulkread data of Dynamixel#1 is available
                dxl_getdata_result = groupBulkRead.isAvailable(motor_parameters[i].ID, motor_parameters[i].present_position_addr, LEN_PRO_PRESENT_POSITION);
                if (dxl_getdata_result != true)
                {
                    // fprintf(stderr, "[ID:%03d] groupBulkRead getdata failed\n", motor_parameters[i].ID);
                    continue;
                }
                // Get present position value
                motor_parameters[i].current_position = groupBulkRead.getData(motor_parameters[i].ID, motor_parameters[i].present_position_addr, LEN_PRO_PRESENT_POSITION);
                if (abs(motor_parameters[i].current_position - motor_parameters[i].goal_position) > minimum_value_for_movement)
                {
                    all_joints_completion = false;
                }
                motor_parameters[i].current_angle = motor_parameters[i].reverse * Convert_Value_to_Radian(motor_parameters[i].current_position, i);
                updated_joints++;
                //  printf("[ID:%03d] Goal: %d,  Current Position : %d, Current angle: %f\n", motor_parameters[i].ID, motor_parameters[i].goal_position, motor_parameters[i].current_position, motor_parameters[i].current_angle);
            }
            //ROS_INFO("UPDATED JOINTS %d", updated_joints);
            //std::this_thread::sleep_for(std::chrono::milliseconds(1));
            Publish_Joints();
            joints_intents_count++;
        } while (!all_joints_completion && death_man_state && ros::ok() && joints_intents_count < 0);
        // if (death_man_state)
        //    Stop_Motors();
        return true;
    }
    bool Ping_Motors()
    {
        int dxl_comm_result = COMM_TX_FAIL;
        // Try to broadcast ping the Dynamixel
        uint16_t dxl_model_number;
        uint8_t dxl_error = 0;
        for (auto motor : motor_parameters)
        {
            int ping_cnt = 0;
            do
            {
                dxl_comm_result = packetHandler->ping(portHandler, motor.ID, &dxl_model_number, &dxl_error);
                if (dxl_comm_result != COMM_SUCCESS)
                {
                    ROS_ERROR("[ID: %d] Ping failure: %s!\n", motor.ID, packetHandler->getTxRxResult(dxl_comm_result));
                    // return false;
                    ping_cnt++;
                }
                else
                {
                    printf("[ID: %d] Ping success! model number: %d\n", motor.ID, dxl_model_number);
                }

            } while (dxl_comm_result != COMM_SUCCESS && ros::ok() && ping_cnt < 5);
        }
        return true;
    }
    void Stop_Motors()
    { //stop movements by sending current joints  position
        bool dxl_addparam_result = false;
        bool dxl_getdata_result = false;
        int dxl_comm_result = COMM_TX_FAIL;
        int stop_cnt = 0;
        do
        {
            dxl_comm_result = groupBulkRead.txRxPacket();
            if (dxl_comm_result != COMM_SUCCESS)
            {
               // printf("Error Stopping: %s, retrying!\n", packetHandler->getTxRxResult(dxl_comm_result));
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                stop_cnt++;
            }
        } while (dxl_comm_result != COMM_SUCCESS && ros::ok() && stop_cnt < 5);

        for (int i = 0; i < number_motors; i++)
        {

            dxl_getdata_result = groupBulkRead.isAvailable(motor_parameters[i].ID, motor_parameters[i].present_position_addr, LEN_PRO_PRESENT_POSITION);
            if (dxl_getdata_result != true)
            {
                //fprintf(stderr, "[ID:%03d] groupBulkRead getdata failed\n", motor_parameters[i].ID);
                return;
            }
            motor_parameters[i].current_position = groupBulkRead.getData(motor_parameters[i].ID, motor_parameters[i].present_position_addr, LEN_PRO_PRESENT_POSITION);
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

                dxl_addparam_result = groupBulkWrite.addParam(motor_parameters[i].ID, motor_parameters[i].goal_position_addr, LEN_PRO_GOAL_POSITION, param_goal_position);
                if (dxl_addparam_result != true)
                {
                  //  fprintf(stderr, "[ID:%03d] stopping mottors groupBulkWrite addparam failed", motor_parameters[i].ID);
                    return;
                }
            }

            dxl_comm_result = groupBulkWrite.txPacket();
            if (dxl_comm_result != COMM_SUCCESS)
                printf("Stopping motors: No success reading,  %s\n", packetHandler->getTxRxResult(dxl_comm_result));
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
            int disable_cnt = 0;
            do
            {
                disable_cnt++;
                dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, motor.ID, motor.torque_enable_addr, 0, &dxl_error);
            } while (dxl_comm_result != COMM_SUCCESS && ros::ok() && disable_cnt < 30);
            if (dxl_comm_result != COMM_SUCCESS)
            {
               // printf("[ID: %03d] No success disabling, %s\n", motor.ID, packetHandler->getTxRxResult(dxl_comm_result));
                return false;
            }
            else if (dxl_error != 0)
            {
               // printf("[ID: %03d] Error disabling, %s\n", motor.ID, packetHandler->getRxPacketError(dxl_error));
                return false;
            }
            else
            {
                //   printf("DXL#%d has been successfully disabled \n", motor.ID);
            }
        }
        return true;
    }
    bool Disable_Torques_Bulk()
    {
        bool dxl_addparam_result = false;
        bool dxl_getdata_result = false;
        int dxl_comm_result = COMM_TX_FAIL;
        int goal = 0;
        uint8_t param_disable_value[1];
        param_disable_value[0] = DXL_LOBYTE(DXL_LOWORD(goal));
        for (auto motor : motor_parameters)
        {
            if (motor.ID == 22)
                continue;
            dxl_addparam_result = groupBulkWrite.addParam(motor.ID, motor.torque_enable_addr, 1, param_disable_value);
            if (dxl_addparam_result != true)
            {
                //fprintf(stderr, "[ID:%03d] Disable Motors Torque Bulk: groupBulkWrite addparam failed", motor.ID);
                return false;
            }
        }
        dxl_comm_result = groupBulkWrite.txPacket();
        groupBulkWrite.clearParam();
        if (dxl_comm_result != COMM_SUCCESS)
        {
            // printf("Write Disable Torques Bulk: No success reading,  %s\n", packetHandler->getTxRxResult(dxl_comm_result));
            return false;
        }
        return true;
    }
    bool Disable_Individual_Torque(int index)
    {
        uint8_t dxl_error = 0;
        int dxl_comm_result = COMM_TX_FAIL;

        bool dxl_addparam_result = false;
        auto motor = motor_parameters[index];
        int disable_cnt = 0;
        do
        {
            disable_cnt++;
            dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, motor.ID, motor.torque_enable_addr, 0, &dxl_error);
        } while (dxl_comm_result != COMM_SUCCESS && ros::ok() && disable_cnt < 30);

        if (dxl_comm_result != COMM_SUCCESS)
        {
            printf("[ID: %03d] No success disabling individual, %s\n", motor.ID, packetHandler->getTxRxResult(dxl_comm_result));
            return false;
        }
        else if (dxl_error != 0)
        {
            printf("[ID: %03d] Error disabling, %s\n", motor.ID, packetHandler->getRxPacketError(dxl_error));
            return false;
        }
        else
        {
#ifdef DEBUG
            printf("DXL#%d has been successfully disabled \n", motor.ID);
#endif
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
