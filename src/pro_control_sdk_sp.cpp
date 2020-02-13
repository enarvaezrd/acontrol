/*
    Pro arm control node
    Dynamixel Pro series: H54, H42, H42P
    Position control
    Eduardo Narvaez
*/

#include "pro_control_sdk_sp.hpp"

#define DEVICENAME "/dev/ttyUSB0"
#define PROTOCOL_VERSION 2.0
// Data Byte Length
#define LEN_PRO_LED_RED                 1
#define LEN_PRO_GOAL_POSITION           4
#define LEN_PRO_PRESENT_POSITION        4

double protocol_version = 2.0;
int baud_rate = 1000000;
double packet_timeout = 16; //ms
int minimum_value_for_movement = 10;
dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
dynamixel::GroupBulkRead groupBulkRead(portHandler, packetHandler);
dynamixel::GroupBulkWrite groupBulkWrite(portHandler, packetHandler);
bool motors_stoped = true;
vector<motor_params> motor_parameters;
int number_motors = 0;
ros::Publisher joint_states_pub;

bool Open_Port();
bool Ping_Motors();
bool Prepare_Bulk_Reader();
void Stop_Motors();
bool Disable_Torques_Bulk();
bool Enable_Torques_Bulk();
void Read_and_Publish_Joints();
void Publish_Joints();
int Convert_Radian_to_Value(double radian, int motor_index);
int Convert_Angle_to_Value(double angle, int motor_index);
bool Disable_Individual_Torque(int index);
bool Bulk_Write_Position_Goals(vector<double> position_goals_rad);
void Close_Port();

double Convert_Value_to_Radian(int value, int motor_index);

control_msgs::FollowJointTrajectoryGoal trajectory_goal;
bool new_trajectory_received = false;
void Trajectory_Handler(const control_msgs::FollowJointTrajectoryGoal &traj_goal)
{
    trajectory_goal = traj_goal;
    new_trajectory_received = true;
}

sensor_msgs::Joy joystick_msg;
bool death_man_state = false;
bool new_joy_message_received = false;
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
int main(int argc, char **argv)
{
    ros::init(argc, argv, "pro_arm_controller_sdk");

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    ros::NodeHandle ros_node_handler;
    ros::Rate loop_rate(45);
    bool service_state = ros::service::exists("/dynamixel_workbench_pro/dynamixel_command", true);
    cout << "service state " << service_state << endl;
    ros::Subscriber sub_Trajectory = ros_node_handler.subscribe("/robot1/arm_general/goal_command", 1, Trajectory_Handler);
    joint_states_pub = ros_node_handler.advertise<sensor_msgs::JointState>("/dynamixel_ed_pro_control/joint_states", 1);
    /*------------------ Motors settings  -------------------*/
    vector<int> motor_ids{11, 22, 33};
    vector<string> motor_types{"pro42", "pro54", "pro42_plus"};
    vector<pair<int, int>> min_max_values_{make_pair(-303750, 303750), make_pair(-175000, 175000), make_pair(-228000, 228000)};
    vector<double> resolutions_{303750.0 / 360.0, 501923.0 / 360.0, 607500.0 / 360.0};
    vector<double> offsets_{0.0, 0.0, 0.0};
    vector<int> velocity_limits_{1947, 3218, 641}; //PRO-H54  => 0.00199234 * 1954  = 3.89303236 RPM
                                                   //PRO-H42P =>       0.01 * 389.3 = 3.89303 RPM
                                                   //PRO-H42  =>  0.0032928 * 1518  = 5 RPM
    vector<int> mov_reversal_{-1, 1, 1};           //antihorary inversion
    /* ----------------------------------------------------- */

    ros::Subscriber sub_joystick = ros_node_handler.subscribe("/joy", 1, Joy_Handler); ///robot1/robotnik_base_control/odom
    ros::AsyncSpinner spinner(2);
    spinner.start();

    for (int i = 0; i < motor_ids.size(); i++)
    {
        motor_params motor;

        motor.ID = (uint8_t) (motor_ids[i]);
        motor.motor_limits = min_max_values_[i];
        motor.protocol_version = protocol_version;
        motor.offset = offsets_[i];
        motor.resolution = resolutions_[i];
        motor.moving_threshold = minimum_value_for_movement;
        motor.velocity_limit = velocity_limits_[i];
        motor.reverse = mov_reversal_[i];

        if (motor_types[i] == "pro42_plus"||motor_types[i] == "pro42") //Firmware (A) version
        {
            motor.torque_enable_addr = (uint8_t)(512);
            motor.led_red_addr = (uint8_t)(513);
            motor.goal_position_addr = (uint8_t)(564);
            motor.present_position_addr = (uint8_t)(580);
            motor.velocity_limit_addr = (uint8_t)(44);
        }
        motor_parameters.push_back(motor);
        number_motors = motor_parameters.size();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    // Open port
    if (!Open_Port())
    {
        return 0;
    }
    Ping_Motors();
    int reader_count = 0;
    bool reader_flag = false;
    while (!reader_flag && reader_count < 100)
    {
        reader_flag = Prepare_Bulk_Reader();
        reader_count++;
        ROS_INFO("NO POSSIBLE TO READ FROM MOTORS");
    }
    if (!reader_flag)
    {
        ROS_INFO("NO POSSIBLE TO READ FROM MOTORS-EXIT");
        return 0;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    bool torque_enabled = false;
    bool motor_send_positions;
    bool joystick_state;
    vector<double> old_motor_positions_{0.0, 0.0, 0.0};
    auto tic_clock = std::chrono::high_resolution_clock::now();
    while (ros::ok())
    {
        // DXL_Handler.Ping_Motors();
        joystick_state = false;

        if (!new_joy_message_received || !new_trajectory_received)
        {
            joystick_state = false; //just to bypass the first iterations when joy is not available
        }
        else if (!death_man_state)
        {
            joystick_state = false;
        }
        else
        {
            joystick_state = true;
        }
        if (!joystick_state)
        {
            //ROS_INFO("STOPPING %d", joystick_state);
            if (torque_enabled)
            {
                Stop_Motors();
                Disable_Torques_Bulk();
                torque_enabled = false;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            ros::spinOnce();
            loop_rate.sleep();
            continue;
        }
        else
        {
            if (!torque_enabled)
            {
                torque_enabled = Enable_Torques_Bulk();
                ROS_INFO("torque %d", torque_enabled);
            }
        }

        vector<double> motor_positions_{trajectory_goal.trajectory.points[0].positions[0],
                                        trajectory_goal.trajectory.points[0].positions[1],
                                        trajectory_goal.trajectory.points[0].positions[2]};
        tic_clock = std::chrono::high_resolution_clock::now();
        Read_and_Publish_Joints();
        auto toc_clock = std::chrono::high_resolution_clock::now();
        auto elapsed_c = std::chrono::duration_cast<std::chrono::microseconds>(toc_clock - tic_clock);
        ROS_INFO("TIME %d", (int)(elapsed_c.count()));
        motor_send_positions = false;
        int limit_iterations_cnt = 0;
        //  if (!motor_positions_comparison(old_motor_positions_, motor_positions_))
        // {
        while (!motor_send_positions && limit_iterations_cnt < 2)
        {

            //tic_clock = std::chrono::high_resolution_clock::now();
            motor_send_positions = Bulk_Write_Position_Goals(motor_positions_);
            //auto toc_clock = std::chrono::high_resolution_clock::now();
            // auto elapsed_c = std::chrono::duration_cast<std::chrono::microseconds>(toc_clock - tic_clock);
            // ROS_INFO("TIME %d, %d", elapsed_c.count(),limit_iterations_cnt);
            limit_iterations_cnt++;
        }
        // }
        // else
        //{
        //    std::cout << "[Pro Arm SDK] Motor positions same as previous request\n";
        // }

        old_motor_positions_ = motor_positions_;
        loop_rate.sleep();
        ros::spinOnce();
    }
    Disable_Individual_Torque(0);
    Close_Port();
    return 0;
}

bool Open_Port()
{
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
bool Prepare_Bulk_Reader()
{
    ROS_INFO("PROTTTTTVER %d", (int)(groupBulkRead.getPacketHandler()->getProtocolVersion()));
    //groupBulkRead.clearParam();
    bool dxl_addparam_result = false;
    for (auto motor : motor_parameters)
    {

        dxl_addparam_result = groupBulkRead.addParam(motor.ID, motor.present_position_addr, (uint16_t)(2));
        ROS_INFO("PREPARING READER JOINTS ID: %d, Addr: %d", motor.ID, motor.present_position_addr);
        if (dxl_addparam_result != COMM_SUCCESS)
        {
            ROS_ERROR("[ID:%03d] Preparing Reader, grouBulkRead addparam failed", motor.ID);
            return false;
        }
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
            printf("Error Stopping: %s, retrying!\n", packetHandler->getTxRxResult(dxl_comm_result));
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            stop_cnt++;
        }
    } while (dxl_comm_result != COMM_SUCCESS && ros::ok() && stop_cnt < 5);

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
                fprintf(stderr, "[ID:%03d] stopping mottors groupBulkWrite addparam failed", motor_parameters[i].ID);
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
        ROS_ERROR("Write Enabling Value: No success reading,  %s\n", packetHandler->getTxRxResult(dxl_comm_result));
        return false;
    }
    return true;
}
void Read_and_Publish_Joints()
{
    int joints_intents_count = 0;
    int updated_joints = 0;

    int reading_count = 0;
    bool dxl_comm_result = false;
    do
    {
        dxl_comm_result = groupBulkRead.txRxPacket();
        if (dxl_comm_result != COMM_SUCCESS)
        {
            //printf("Error write position goals txRxPacket: %s, retrying! joy state: %d \n", packetHandler->getTxRxResult(dxl_comm_result), death_man_state);
            //std::this_thread::sleep_for(std::chrono::milliseconds(1));
            reading_count++;
        }

    } while (dxl_comm_result != COMM_SUCCESS && reading_count < 200 && ros::ok());

    if (!dxl_comm_result)
    {
        return;

        ROS_INFO("UPDATED JOINTS %d", 0);
    }

    for (int i = 0; i < number_motors; i++)
    {
        // Check if groupbulkread data of Dynamixel#1 is available
        bool dxl_getdata_result = groupBulkRead.isAvailable(motor_parameters[i].ID, motor_parameters[i].present_position_addr, (uint16_t)(2));
        ROS_INFO("READING JOINTS ID: %d, Addr: %d", motor_parameters[i].ID, motor_parameters[i].present_position_addr);
        if (dxl_getdata_result != true)
        {
            continue;
        }
        // Get present position value
        motor_parameters[i].current_position = groupBulkRead.getData(motor_parameters[i].ID, motor_parameters[i].present_position_addr, (uint16_t)(2));

        motor_parameters[i].current_angle = motor_parameters[i].reverse * Convert_Value_to_Radian(motor_parameters[i].current_position, i);
        updated_joints++;
    }
    ROS_INFO("UPDATED JOINTS %d", updated_joints);
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

        dxl_addparam_result = groupBulkWrite.addParam(motor_parameters[i].ID, motor_parameters[i].goal_position_addr, 4, param_goal_position);
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
                printf("Error write position goals txRxPacket: %s, retrying! joy state: %d \n", packetHandler->getTxRxResult(dxl_comm_result), death_man_state);
                //std::this_thread::sleep_for(std::chrono::milliseconds(1));
                reading_count++;
            }

        } while (dxl_comm_result != COMM_SUCCESS && death_man_state && reading_count > 10 && ros::ok());

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
            dxl_getdata_result = groupBulkRead.isAvailable(motor_parameters[i].ID, motor_parameters[i].present_position_addr, 4);
            if (dxl_getdata_result != true)
            {
                // fprintf(stderr, "[ID:%03d] groupBulkRead getdata failed\n", motor_parameters[i].ID);
                continue;
            }
            // Get present position value
            motor_parameters[i].current_position = groupBulkRead.getData(motor_parameters[i].ID, motor_parameters[i].present_position_addr, 4);
            if (abs(motor_parameters[i].current_position - motor_parameters[i].goal_position) > minimum_value_for_movement)
            {
                all_joints_completion = false;
            }
            motor_parameters[i].current_angle = motor_parameters[i].reverse * Convert_Value_to_Radian(motor_parameters[i].current_position, i);
            updated_joints++;
            //  printf("[ID:%03d] Goal: %d,  Current Position : %d, Current angle: %f\n", motor_parameters[i].ID, motor_parameters[i].goal_position, motor_parameters[i].current_position, motor_parameters[i].current_angle);
        }
        ROS_INFO("UPDATED JOINTS %d", updated_joints);
        //std::this_thread::sleep_for(std::chrono::milliseconds(1));
        Publish_Joints();
        joints_intents_count++;
    } while (!all_joints_completion && death_man_state && ros::ok() && joints_intents_count < 0);
    // if (death_man_state)
    //    Stop_Motors();
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