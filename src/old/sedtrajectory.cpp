#include "edtrajectory.h"
#include<iostream>
double PI=3.141592654;
namespace katana_tutorials
{

FollowJointTrajectoryClient::FollowJointTrajectoryClient() :
    traj_client_("/robot1/arm_arm_controller/follow_joint_trajectory", true), got_joint_state_(false), spinner_(0)
{
    /*joint_names_.push_back("arm_1_joint");
    joint_names_.push_back("arm_2_joint");
    joint_names_.push_back("arm_3_joint");
    joint_names_.push_back("arm_4_joint");
    joint_names_.push_back("arm_5_joint");
    joint_names_.push_back("arm_6_joint"); PARA ARM1*/

    joint_names_.push_back("Shoulder_Joint");
    joint_names_.push_back("Arm_Joint");
    joint_names_.push_back("Forearm_Joint");
    joint_names_.push_back("Wrist_1_Joint");
    joint_names_.push_back("Wrist_2_Joint");
    joint_names_.push_back("Hand_Joint"); //PARA MICO

  joint_state_sub_ = nh_.subscribe("/robot1/joint_states", 1, &FollowJointTrajectoryClient::jointStateCB, this);
   spinner_.start();

  // wait for action server to come up
  while (!traj_client_.waitForServer(ros::Duration(1.0)))
  {
    ROS_INFO("Waiting for the follow_joint_trajectory server");
  }
}

FollowJointTrajectoryClient::~FollowJointTrajectoryClient()
{
}

void FollowJointTrajectoryClient::jointStateCB(const sensor_msgs::JointState::ConstPtr &msg)
{
  std::vector<double> ordered_js;

  ordered_js.resize(joint_names_.size());

  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    bool found = false;
    for (size_t j = 0; j < msg->name.size(); ++j)
    {
      if (joint_names_[i] == msg->name[j])
      {
        ordered_js[i] = msg->position[j];
        found = true;
        break;
      }
    }
    if (!found)
      return;
  }

  ROS_INFO_ONCE("Got joint state!");
  current_joint_state_ = ordered_js;
  got_joint_state_ = true;
}

//! Sends the command to start a given trajectory
void FollowJointTrajectoryClient::startTrajectory(control_msgs::FollowJointTrajectoryGoal goal)
{
  // When to start the trajectory: 1s from now
  goal.trajectory.header.stamp = ros::Time::now() ;

  //std::cout<<goal;
  traj_client_.sendGoal(goal);
}

control_msgs::FollowJointTrajectoryGoal FollowJointTrajectoryClient::makeArmUpTrajectory(std::vector<double> joints_obj)
{
  const size_t NUM_TRAJ_POINTS = 4;
  const size_t NUM_JOINTS = 5;

  // positions after calibration
  std::vector<double> calibration_positions(NUM_JOINTS);
  calibration_positions[0] = -2.96;
  calibration_positions[1] = 2.14;
  calibration_positions[2] = -2.16;
  calibration_positions[3] = -1.97;
  calibration_positions[4] = -2.93;

  // arm pointing straight up
  std::vector<double> straight_up_positions(NUM_JOINTS);
  straight_up_positions[0] = 0.0;
  straight_up_positions[1] = 1.57;
  straight_up_positions[2] = 0.0;
  straight_up_positions[3] = 0.0;
  straight_up_positions[4] = 0.0;

  trajectory_msgs::JointTrajectory trajectory;

  for (ros::Rate r = ros::Rate(10); !got_joint_state_; r.sleep())
  {
    ROS_DEBUG("waiting for joint state...");

    if (!ros::ok())
      exit(-1);
  }

  // First, the joint names, which apply to all waypoints
  trajectory.joint_names = joint_names_;

  trajectory.points.resize(NUM_TRAJ_POINTS);

  // trajectory point:
  int ind = 0;
  trajectory.points[ind].time_from_start = ros::Duration(5 * ind);
  trajectory.points[ind].positions = current_joint_state_;

  // trajectory point:
  ind++;
  trajectory.points[ind].time_from_start = ros::Duration(5 * ind);
  trajectory.points[ind].positions = calibration_positions;

  // trajectory point:
  ind++;
  trajectory.points[ind].time_from_start = ros::Duration(5 * ind);
  trajectory.points[ind].positions = straight_up_positions;

  // trajectory point:
  ind++;
  trajectory.points[ind].time_from_start = ros::Duration(5 * ind);
  trajectory.points[ind].positions.resize(NUM_JOINTS);
  trajectory.points[ind].positions = calibration_positions;

  //  // all Velocities 0
  //  for (size_t i = 0; i < NUM_TRAJ_POINTS; ++i)
  //  {
  //    trajectory.points[i].velocities.resize(NUM_JOINTS);
  //    for (size_t j = 0; j < NUM_JOINTS; ++j)
  //    {
  //      trajectory.points[i].velocities[j] = 0.0;
  //    }
  //  }

  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory = trajectory;
  return goal;
}

//! Returns the current state of the action
actionlib::SimpleClientGoalState FollowJointTrajectoryClient::getState()
{
  return traj_client_.getState();
}

} /* namespace katana_tutorials */

double err_x,err_y,p_err_x,p_err_y,ctrl_x,ctrl_y;
double de_dt,e_dt=0;
double c_x,c_y;
float Kp=0.01,Ki=0.01,Kd=0.01;
int rate=3.0;
float period=1+1/rate;
double pose_x=0.0,pose_z=0.0,pose_y=0.0;
void Pose_Handler(const geometry_msgs::PoseStamped& pose_quad)
{
    pose_x=pose_quad.pose.position.x;
    pose_y=pose_quad.pose.position.y;
    pose_z=pose_quad.pose.position.z;
}

struct quat {
    float x,y,z,w;
}quaternion;

struct quat toQuaternion(double pitch, double roll, double yaw)
{
        quat q;
        double t0 = std::cos(yaw * 0.5);
        double t1 = std::sin(yaw * 0.5);
        double t2 = std::cos(roll * 0.5);
        double t3 = std::sin(roll * 0.5);
        double t4 = std::cos(pitch * 0.5);
        double t5 = std::sin(pitch * 0.5);

        q.w = t0 * t2 * t4 + t1 * t3 * t5;
        q.x = t0 * t3 * t4 - t1 * t2 * t5;
        q.y = t0 * t2 * t5 + t1 * t3 * t4;
        q.z = t1 * t2 * t4 - t0 * t3 * t5;
        return q;
}


int main(int argc, char** argv)
{
    // Init the ROS node
    ros::init(argc, argv, "follow_joint_trajectory_client");
    ros::NodeHandle nh; std::cout<<"asdf"<<std::endl;
    katana_tutorials::FollowJointTrajectoryClient arm;
    //PARA CINEMATICA INVERSA:
    std::cout<<"asdf"<<std::endl;
    robot_model_loader::RobotModelLoader robot_model_loader("/robot1/robot_description");
    std::cout<<"asdf"<<std::endl;
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    //MOVEIT GROUP
      std::cout<<"asdf"<<std::endl;


     /* moveit::planning_interface::MoveGroup::Options opts("arm");
      opts.node_handle_ = nh;
      opts.robot_description_="/robot1/robot_description";
      opts.group_name_="arm";
      opts.robot_model_=kinematic_model; std::cout<<"asdf"<<std::endl;
      moveit::planning_interface::MoveGroup group(opts);
*/
   //
      std::cout<<"asdfdddd"<<std::endl;



    std::cout<<"asdf"<<std::endl;
    //ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
    std::cout<<"asdf"<<std::endl;
    kinematic_state->setToDefaultValues();
    std::cout<<"asdf"<<std::endl;
    const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("arm");
       std::cout<<"asdf"<<std::endl;

    std::vector<double> joint_values;

     geometry_msgs::Pose pose_test;
     pose_test.orientation.x = 0.0;
     pose_test.orientation.y = 1.0;
     pose_test.orientation.w = 0.0;
     pose_test.orientation.z = 0.0;
     pose_test.position.x = 0.1;
     pose_test.position.y = 0.0;
     pose_test.position.z = 0.9;

    //FUNCION DE EXTRACCION DE LA CINEMATICA INVERSA, PASAR LA POSE Y DA COMO RESULTADO LOS ANGULOS (JOINTS)
    bool found_ik = kinematic_state->setFromIK(joint_model_group, pose_test, 10, 0.1);
    if (found_ik)
    {      kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);    }
    else
    {      ROS_INFO("No se encontro una solucion IK");   }

    const std::vector<std::string> &joint_names = joint_model_group->getJointModelNames();

    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
    for(std::size_t i = 0; i < joint_names.size(); ++i)
    {
      ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    }

moveit::planning_interface::MoveGroupInterface group("arm");
 group.setPlannerId("RRTkConfigDefault");//PRMstarkConfigDefault---RRTConnectkConfigDefault--RRTkConfigDefault--PRMkConfigDefault--RRTstarkConfigDefault
    ROS_INFO("Planning frame: %s", group.getPlanningFrame().c_str());
    ROS_INFO("End Effector link: %s", group.getEndEffectorLink().c_str());

    //JOINTS
    std::vector<double> group_variable_values;
    group.getCurrentState()->copyJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), group_variable_values);

    group_variable_values[0] = 0.0;
    group_variable_values[1] = -PI/2;
    group_variable_values[2] = PI/2;
    group_variable_values[3] = 0.0;
    group_variable_values[4] = 0.0;
    group_variable_values[5] = 0.0;

    group.setJointValueTarget(group_variable_values);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = group.plan(my_plan); //Llamada de Planificacion en move_group

        //PARA ENVIAR TRAYECTORIA A GAZEBO
        control_msgs::FollowJointTrajectoryGoal goaled;
        goaled.trajectory = my_plan.trajectory_.joint_trajectory;
        arm.startTrajectory(goaled);//Inicio de trayectoria en GAZEBO

         sleep(5.0);
         std::cout<<group.getCurrentPose().pose;
        //group.setGoalTolerance(0.03);
         //group.setGoalOrientationTolerance(orientation_tolerance);

     //PATH CONSTRAINTS ORIENTATION
  /*  moveit_msgs::OrientationConstraint ocm;
     ocm.link_name = "Hand_Link";
     ocm.header.frame_id = "odom_combined";
     ocm.orientation.w = 1.0;
     ocm.absolute_x_axis_tolerance = 0.1;
     ocm.absolute_y_axis_tolerance = 0.1;
     ocm.absolute_z_axis_tolerance = 0.1;
     ocm.weight = 1.0;

     test_constraints.orientation_constraints.push_back(ocm);*/
         moveit_msgs::Constraints test_constraints;
     //PATH CONSTRAINTS JOINT
     moveit_msgs::JointConstraint pcm1;
     pcm1.joint_name = "Shoulder_Joint";
     pcm1.position = 0.0;
     pcm1.tolerance_above = PI/4;
     pcm1.tolerance_below = PI/4;
     pcm1.weight = 1.0;
     test_constraints.joint_constraints.push_back(pcm1);
     //PATH CONSTRAINTS JOINT
     moveit_msgs::JointConstraint pcm;
     pcm.joint_name = "Arm_Joint";
     pcm.position = -PI/2;//Entre  1.2---1.6
     pcm.tolerance_above = PI/4;
     pcm.tolerance_below = PI/4;
     pcm.weight = 2.0;
     test_constraints.joint_constraints.push_back(pcm);
     //PATH CONSTRAINTS JOINT
     moveit_msgs::JointConstraint pcm3;
     pcm3.joint_name = "Forearm_Joint";
     pcm3.position = PI/2;
     pcm3.tolerance_above = PI/4;
     pcm3.tolerance_below = PI/4;
     pcm3.weight = 2.0;
     test_constraints.joint_constraints.push_back(pcm3);
     //PATH CONSTRAINTS JOINT
     moveit_msgs::JointConstraint pcm4;
     pcm4.joint_name = "Wrist_1_Joint";
     pcm4.position = 0;
     pcm4.tolerance_above = PI;
     pcm4.tolerance_below = PI;
     pcm4.weight = 1.0;
     test_constraints.joint_constraints.push_back(pcm4);
     //PATH CONSTRAINTS JOINT
     moveit_msgs::JointConstraint pcm5;
     pcm5.joint_name = "Wrist_2_Joint";
     pcm5.position = 0.0;
     pcm5.tolerance_above = 1.0;
     pcm5.tolerance_below = 1.0;
     pcm5.weight = 1.0;
     test_constraints.joint_constraints.push_back(pcm5);
     //SET CONSTRAINTS
     group.setPathConstraints(test_constraints);




     robot_state::RobotState start_state(*group.getCurrentState());
     geometry_msgs::Pose start_pose2;
     start_pose2.orientation.y = 1.0;
     start_pose2.position.x = 0.2;
     start_pose2.position.y = 0.05;
     start_pose2.position.z = 0.6;
     const robot_state::JointModelGroup *joint_model_group1 = start_state.getJointModelGroup(group.getName());
     start_state.setFromIK(joint_model_group1, start_pose2);
     //group.setStartState(start_state);

     geometry_msgs::Pose target_pose1;
     target_pose1.orientation.x = -0.707;
     target_pose1.orientation.y = -0.707;
     target_pose1.position.x = 0.18;
     target_pose1.position.y = 0.0;
     target_pose1.position.z = 0.9477;
     group.setPoseTarget(target_pose1);
     success = group.plan(my_plan);

     ROS_INFO("Visualizing plan 3 (constraints) %s",success?"":"FAILED");
         //PARA ENVIAR TRAYECTORIA A GAZEBO
         goaled.trajectory = my_plan.trajectory_.joint_trajectory;
         arm.startTrajectory(goaled);//Inicio de trayectoria en GAZEBO

          sleep(3.0);
          std::vector<double> group_variable_values1;
          group.getCurrentState()->copyJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), group_variable_values1);
          //std::cout<<group_variable_values1[0]<<"--"<<group_variable_values1[1]<<"--"<<group_variable_values1[2]<<"--"<<group_variable_values1[3]<<"--"<<group_variable_values1[4]<<"--"<<group_variable_values1[5]<<"--"<<std::endl;


   while (ros::ok())
   {
       std::vector<double> joint_valuesg;
       //group.setPathConstraints(test_constraints);
       // group.setWorkspace(0,-0.3,0.85,0.38,0.3,0.95);

       geometry_msgs::Pose target_pose3 =  target_pose1;
       target_pose3.position.y += 0.01;

       group.setJointValueTarget(target_pose3,"Hand_Link");
       success = group.plan(my_plan);
       //PARA ENVIAR TRAYECTORIA A GAZEBO
           goaled.trajectory = my_plan.trajectory_.joint_trajectory;
           arm.startTrajectory(goaled);//Inicio de trayectoria en GAZEBO
       sleep(1.0);

    ros::spinOnce();
       target_pose3.position.x += 0.01;

       group.setJointValueTarget(target_pose3,"Hand_Link");
       success = group.plan(my_plan);
       //PARA ENVIAR TRAYECTORIA A GAZEBO
           goaled.trajectory = my_plan.trajectory_.joint_trajectory;
           arm.startTrajectory(goaled);//Inicio de trayectoria en GAZEBO
       sleep(1.0);
    ros::spinOnce();
       target_pose3.position.y -= 0.01;

       group.setJointValueTarget(target_pose3,"Hand_Link");
       success = group.plan(my_plan);
       //PARA ENVIAR TRAYECTORIA A GAZEBO
           goaled.trajectory = my_plan.trajectory_.joint_trajectory;
           arm.startTrajectory(goaled);//Inicio de trayectoria en GAZEBO
       sleep(1.0);
    ros::spinOnce();
       target_pose3.position.x -= 0.01;

       group.setJointValueTarget(target_pose3,"Hand_Link");
       success = group.plan(my_plan);
       //PARA ENVIAR TRAYECTORIA A GAZEBO
           goaled.trajectory = my_plan.trajectory_.joint_trajectory;
           arm.startTrajectory(goaled);//Inicio de trayectoria en GAZEBO
       sleep(1.0);

        ros::spinOnce();
    //loop_rate.sleep();
 }





}

