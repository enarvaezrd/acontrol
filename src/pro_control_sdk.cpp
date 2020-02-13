/*
    Pro arm control node
    Dynamixel Pro series: H54, H42, H42P
    Position control
    Eduardo Narvaez
*/

#include "pro_control_sdk.hpp"

#define DEVICENAME "/dev/ttyUSB0"
#define PROTOCOL_VERSION 2.0



control_msgs::FollowJointTrajectoryGoal trajectory_goal;
bool new_trajectory_received = false;
void Trajectory_Handler(const control_msgs::FollowJointTrajectoryGoal &traj_goal)
{
    trajectory_goal = traj_goal;
    new_trajectory_received = true;
}

bool motor_positions_comparison(vector<double> positions_1, vector<double> positions_2)
{
    int size1 = positions_1.size();
    int size2 = positions_2.size();
    double max_diff = 0.0001;
    if (size1 == size1)
    {
        for (int i = 0; i < size1; i++)
        {
            if (positions_1[i] - max_diff > positions_2[i] || positions_2[i] > positions_1[i] + max_diff)
            {
                return false;
            }
        }
        return true;
    }
    else
    {
        ROS_ERROR("the motor positions size is not the same pro_arm_sdk\n");
        return true;
    }
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

    /*------------------ Motors settings  -------------------*/
    vector<int> motor_ids{11, 22, 33};
    vector<string> motor_types{"pro42", "pro54", "pro42_plus"};
    vector<pair<int, int>> min_max_values_{make_pair(-303750, 303750), make_pair(-175000, 175000), make_pair(-228000, 228000)};
    vector<double> resolutions_{607500.0 / 360.0, 501923.0 / 360.0, 607500.0 / 360.0};
    vector<double> offsets_{0.0, 0.0, 0.0};
    vector<int> velocity_limits_{500, 3218, 641}; //PRO-H54  => 0.00199234 * 1954  = 3.89303236 RPM
                                                   //PRO-H42P =>       0.01 * 389.3 = 3.89303 RPM
                                                   //PRO-H42  =>  0.01 * 1518  = 5 RPM
    vector<int> mov_reversal_{-1, 1, 1};           //antihorary inversion
    /* ----------------------------------------------------- */

    string device_name = DEVICENAME;
    float protocol_version = 2.0;
    int baud_rate = 1000000;

    double packet_timeout = 8; //ms
    int minimum_value_for_movement = 10;
    Dynamixel_SDK_Handler DXL_Handler(device_name.c_str(), protocol_version, baud_rate, packet_timeout, minimum_value_for_movement);
    ros::Subscriber sub_joystick = ros_node_handler.subscribe("/joy", 1, &Dynamixel_SDK_Handler::Joy_Handler, &DXL_Handler); ///robot1/robotnik_base_control/odom
    ros::AsyncSpinner spinner(2);
    spinner.start();

    for (int i = 0; i < motor_ids.size(); i++)
    {
        motor_params motor;

        motor.ID = motor_ids[i];
        motor.motor_limits = min_max_values_[i];
        motor.protocol_version = protocol_version;
        motor.offset = offsets_[i];
        motor.resolution = resolutions_[i];
        motor.moving_threshold = minimum_value_for_movement;
        motor.velocity_limit = velocity_limits_[i];
        motor.reverse = mov_reversal_[i];

        if  (motor_types[i] == "pro42_plus"||motor_types[i] == "pro42") //Firmware (A) version
        {
            motor.torque_enable_addr = 512;
            motor.led_red_addr = 513;
            motor.goal_position_addr = 564;
            motor.present_position_addr = 580;
            motor.velocity_limit_addr = 44;
        }
        DXL_Handler.Initialize_Motor_Parameters(motor);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    if (!DXL_Handler.Open_Port())
    {
        ROS_ERROR("PRO_CONTROL_SDK Can't Open Port");
        return 0;
    }
    DXL_Handler.Ping_Motors();

    bool write_vel_state = false;
   
  int reader_count = 0;
    bool reader_flag = false;
    while (!reader_flag && reader_count < 100)
    {
        reader_flag = DXL_Handler.Prepare_Bulk_Reader();
        reader_count++;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
     if (!reader_flag)
    {
        ROS_INFO("NO POSSIBLE TO READ FROM MOTORS-EXIT");
        return 0;
    }
    else
    {
        ROS_INFO("MOTOR READER SUCCESS");
    }
    
    /* DETERMINE VELOCITIES IN WINDOWS PROGRAM 
   while (!write_vel_state && ros::ok() && count < 100)
    {
        write_vel_state = DXL_Handler.Write_Velocity_Limits_Bulk();
       
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        //  count++;
    }
    if (!write_vel_state)
    {
        ROS_ERROR("PRO_CONTROL_SDK Can't Write velocity limits! Finished");
        return 0;
    }
   */
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

        if (!DXL_Handler.new_joy_message_received || !new_trajectory_received)
        {
            joystick_state = false; //just to bypass the first iterations when joy is not available
        }
        else if (!DXL_Handler.death_man_state)
        {
            joystick_state = false;
        }
        else
        {
            joystick_state = true;
        }
        if (!joystick_state)
        {
            // ROS_INFO("STOPPING %d", joystick_state);
            if (torque_enabled)
            {
                DXL_Handler.Stop_Motors();
                DXL_Handler.Disable_Torques_Bulk();
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
                torque_enabled = DXL_Handler.Enable_Torques_Bulk();
                ROS_INFO("torque %d", torque_enabled);
            }
        }

        vector<double> motor_positions_{trajectory_goal.trajectory.points[0].positions[0],
                                        trajectory_goal.trajectory.points[0].positions[1],
                                        trajectory_goal.trajectory.points[0].positions[2]};
        
        motor_send_positions = false;
        int limit_iterations_cnt = 0;
        //  if (!motor_positions_comparison(old_motor_positions_, motor_positions_))
        // {
        while (!motor_send_positions && limit_iterations_cnt < 5)
        {

            //tic_clock = std::chrono::high_resolution_clock::now();
            motor_send_positions = DXL_Handler.Bulk_Write_Position_Goals(motor_positions_);
            //auto toc_clock = std::chrono::high_resolution_clock::now();
           // auto elapsed_c = std::chrono::duration_cast<std::chrono::microseconds>(toc_clock - tic_clock);
           // ROS_INFO("TIME %d, %d", elapsed_c.count(),limit_iterations_cnt);
            limit_iterations_cnt++;
        }
       /* tic_clock = std::chrono::high_resolution_clock::now();
        //DXL_Handler.Read_and_Publish_Joints();
        DXL_Handler.Read_Individual_and_Publish_Joints();
        auto toc_clock = std::chrono::high_resolution_clock::now();
        auto elapsed_c = std::chrono::duration_cast<std::chrono::microseconds>(toc_clock - tic_clock);
        ROS_INFO("TIME %d", (int)(elapsed_c.count()));   */    
        // }
        // else
        //{
        //    std::cout << "[Pro Arm SDK] Motor positions same as previous request\n";
        // }

        old_motor_positions_ = motor_positions_;
        loop_rate.sleep();
        ros::spinOnce();
    }
    DXL_Handler.Disable_Individual_Torque(0);
    DXL_Handler.Close_Port();
    return 0;
}