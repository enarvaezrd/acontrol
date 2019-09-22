#include "edtrajectory.h"
#include<iostream>
double PI=3.141592654;
namespace katana_tutorials
{

FollowJointTrajectoryClient::FollowJointTrajectoryClient() :
    traj_client_("/robot1/arm_controller/follow_joint_trajectory", true), got_joint_state_(false), spinner_(0)
{
    joint_names_.push_back("joint1");
    joint_names_.push_back("joint2");
    joint_names_.push_back("joint3");
    joint_names_.push_back("joint4");
    joint_names_.push_back("joint5");
    joint_names_.push_back("joint6");
    joint_names_.push_back("joint2g");//PARA EARM


  joint_state_sub_ = nh_.subscribe("/robot1/joint_states", 1, &FollowJointTrajectoryClient::jointStateCB, this);
   spinner_.start();
  // wait for action server to come up
  while (!traj_client_.waitForServer(ros::Duration(5.0)))
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
{// std::cout<<" Entrada "<<std::endl;
  // When to start the trajectory: 1s from now
    int NUM=7;
  goal.trajectory.header.stamp = ros::Time::now();// + ros::Duration(0.035);
   //std::cout<<" Entrada2 "<<std::endl;
  goal.trajectory.joint_names=joint_names_;
  int sizeP=goal.trajectory.points.size();
 //  std::cout<<goal<<" puntos "<<sizeP<<std::endl;
  for (int i=0;i<sizeP;i++)
  {
      goal.trajectory.points[i].positions.resize(NUM);
      goal.trajectory.points[i].velocities.resize(NUM);
      goal.trajectory.points[i].accelerations.resize(NUM);
      goal.trajectory.points[i].positions[6]= -goal.trajectory.points[i].positions[1]; //Para agregar joint duplicada, se invierte porque esta ubicada de forma opuesta, gemela
      goal.trajectory.points[i].velocities[6]= goal.trajectory.points[i].velocities[1];
      goal.trajectory.points[i].accelerations[6]= goal.trajectory.points[i].accelerations[1];
  }
 if (sizeP>2){goal.trajectory.points[sizeP-1].velocities= goal.trajectory.points[sizeP-2].velocities;
  goal.trajectory.points[sizeP-1].accelerations= goal.trajectory.points[sizeP-2].accelerations;
}
// std::cout<<goal;
  traj_client_.sendGoal(goal);
}

control_msgs::FollowJointTrajectoryGoal FollowJointTrajectoryClient::makeArmUpTrajectory(std::vector<double> joints_obj)
{
  const size_t NUM_TRAJ_POINTS = 1;
  const size_t NUM_JOINTS = 7;

  // positions after calibration
  std::vector<double> mid_positions(NUM_JOINTS);
  mid_positions[0] = joints_obj[0];
  mid_positions[1] = joints_obj[1];
  mid_positions[2] = joints_obj[2];
  mid_positions[3] = joints_obj[3];
  mid_positions[4] = joints_obj[4];
  mid_positions[5] = joints_obj[5];
  mid_positions[6] = -joints_obj[1];

  // arm pointing straight up
  std::vector<double> straight_up_positions(NUM_JOINTS);
  straight_up_positions[0] = joints_obj[0];
  straight_up_positions[1] = joints_obj[1];
  straight_up_positions[2] = joints_obj[2];
  straight_up_positions[3] = joints_obj[3];
  straight_up_positions[4] = joints_obj[4];
  straight_up_positions[5] = joints_obj[5];
  straight_up_positions[6] = -joints_obj[1];

  trajectory_msgs::JointTrajectory trajectory;https://www.facebook.com/ayana.okada.7?fref=pb&hc_location=friends_tab
  for (ros::Rate r = ros::Rate(10); !got_joint_state_; r.sleep())
  {
    ROS_DEBUG("waiting for joint state...");

    if (!ros::ok())
      exit(-1);
  }

double promDiff=10.0;
promDiff=(std::abs(joints_obj[0]-current_joint_state_[0])+std::abs(joints_obj[1]-current_joint_state_[1])+std::abs(joints_obj[2]-current_joint_state_[2])+1.5*std::abs(joints_obj[5]-current_joint_state_[5]))/4;//promedio de diferencia en posicion, joint coordinates
//promDiff=std::abs(joints_obj[0]-current_joint_state_[0]);


//std::cout<<promDiff<<std::endl;

  // First, the joint names, which apply to all waypoints
  trajectory.joint_names = joint_names_;

  trajectory.points.resize(NUM_TRAJ_POINTS);

  // trajectory point:
  int ind = 0;
  float M1=0.5;
  //trajectory.points[ind].time_from_start = ros::Duration(1* ind);
  //trajectory.points[ind].positions = mid_positions;

  // trajectory point:
  //ind++;
  if (promDiff>0.2) {
    promDiff=promDiff*2;
    }

  double Ttimer=0.0+(promDiff*M1);
  trajectory.points[ind].time_from_start = ros::Duration(Ttimer);
  trajectory.points[ind].positions.resize(NUM_JOINTS);
  trajectory.points[ind].positions = mid_positions;

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

struct quat {
    double x,y,z,w;
}quaternion;

struct quat toQuaternion(double yawMark, geometry_msgs::Pose cpose)
{            quat qc;
             qc.x=cpose.orientation.x; qc.y=cpose.orientation.y;
             qc.z=cpose.orientation.z; qc.w=cpose.orientation.w;
             double ysqrc = qc.y * qc.y;
                     double t3c = +2.0 * (qc.w * qc.z + qc.x * qc.y);
                     double t4c = +1.0 - 2.0 * (ysqrc + qc.z * qc.z);
                     double yawc = std::atan2(t3c, t4c);//end effector

                    double yaw=yawMark;

        float yawc1=yawc,yaw1=yaw;
         /*if ( yawc >= 0  )       yawc1 =  yawc - PI ;
            if ( yawc < 0)       yawc1 =  PI + yawc;

            if ( yaw >= 0  )       yaw1 =  -(yaw - PI) ;
            if ( yaw < 0)       yaw1 =  -(PI + yaw);*/

        yaw=yawc1+(yaw1);
        //  yaw=PI/2;
        if (yaw>=(17*PI/15)) {yaw=yaw-(2*PI);}
        else {
        if (yaw<=(-17*PI/15)) yaw=yaw+(2*PI);
        }
        std::cout<<" YAWENDEFF "<<yaw<<std::endl;
        if (yaw>=(5*PI/6.01)) yaw=(5*PI/6.01);
        if (yaw<=(-5*PI/6.01)) yaw=(-5*PI/6.01);
        double diffpe=yaw-yawc1;
        double newyaw;
        if (abs(diffpe)>PI) {newyaw=yawc1+(1.0*diffpe);}
        else {newyaw=yawc1+(0.1*diffpe);}
        yaw =newyaw;
        double roll=-PI/2;
        double  pitch=0;
        quat q1;
        double t0 = std::cos(yaw   * 0.5);
        double t1 = std::sin(yaw   * 0.5);
        double t2 = std::cos(roll  * 0.5);
        double t3 = std::sin(roll  * 0.5);
        double t4 = std::cos(pitch * 0.5);
        double t5 = std::sin(pitch * 0.5);

        q1.w = t0 * t2 * t4 + t1 * t3 * t5;
        q1.x = t0 * t3 * t4 - t1 * t2 * t5;
        q1.y = t0 * t2 * t5 + t1 * t3 * t4;
        q1.z = t1 * t2 * t4 - t0 * t3 * t5;
        return q1;
}

trajectory_msgs::JointTrajectory retimeTrajectory(trajectory_msgs::JointTrajectory from, double factor)
{

   trajectory_msgs::JointTrajectory to = from;

  std::size_t num_traj_points = to.points.size();
  std::size_t num_joints = to.joint_names.size();
 std::cout<< "  ENTRADA1   "<<  num_joints<< " puntos "<< num_traj_points << std::endl;
  double inv_factor = (1.0/factor);

  for (int i=0 ; i < num_traj_points; i++){
    to.points[i].time_from_start *= inv_factor;
    for (size_t j; j < num_joints; ++j){
      to.points[i].velocities[j] *= factor;
    }
  }
  return to;
}

struct quat ConvtoQuaternion(double pitch, double roll, double yaw)
{
        quat q1;
        double t0 = std::cos(yaw * 0.5);
        double t1 = std::sin(yaw * 0.5);
        double t2 = std::cos(roll * 0.5);
        double t3 = std::sin(roll * 0.5);
        double t4 = std::cos(pitch * 0.5);
        double t5 = std::sin(pitch * 0.5);
        q1.w = t0 * t2 * t4 + t1 * t3 * t5;
        q1.x = t0 * t3 * t4 - t1 * t2 * t5;
        q1.y = t0 * t2 * t5 + t1 * t3 * t4;
        q1.z = t1 * t2 * t4 - t0 * t3 * t5;
        return q1;
}

struct quat ConvtoAngles(geometry_msgs::Pose mpose)
{
        quat q1;
        quat q;
                 q.x=mpose.orientation.x; q.y=mpose.orientation.y;
                 q.z=mpose.orientation.z; q.w=mpose.orientation.w;
                 double ysqr = q.y * q.y;
                // roll (x-axis rotation)
                double t00 = +2.0 * (q.w * q.x + q.y * q.z);
                double t11 = +1.0 - 2.0 * (q.x * q.x + ysqr);
                double roll = std::atan2(t00, t11);

                // pitch (y-axis rotation)
                double t22 = +2.0 * (q.w * q.y - q.z * q.x);
                t22 = t22 > 1.0 ? 1.0 : t22;
                t22 = t22 < -1.0 ? -1.0 : t22;
                double pitch = std::asin(t22);

                // yaw (z-axis rotation)
                double t33 = +2.0 * (q.w * q.z + q.x * q.y);
                double t44 = +1.0 - 2.0 * (ysqr + q.z * q.z);
                double yaw = std::atan2(t33, t44);
                q1.x=roll;q1.y=pitch;q1.z=yaw;
        return q1;
}


geometry_msgs::Pose markpose;
int state;

void Mark_Handler(const geometry_msgs::PoseArray& ma)
{
     state = ma.poses.size();
     if ( state ==1){
         markpose = ma.poses[0];
     }
     else{
         state=0;
     }
}

sensor_msgs::PointCloud2 depth_PC;
void Depth_Handler(const sensor_msgs::PointCloud2& dp)
{
    depth_PC=dp;
}

cv_bridge::CvImagePtr DepthImg_CV;

int LandingFlag;

using namespace katana_tutorials;

int main(int argc, char** argv)
{
    // Init the ROS node
    ros::init(argc, argv, "/robot1/follow_joint_trajectory_client");
    ros::NodeHandle nh;
    ros::Rate loop_rate(20);
    katana_tutorials::FollowJointTrajectoryClient arm;
    //MOVEIT GROUP
    moveit::planning_interface::MoveGroupInterface group("Earm");
    ros::Subscriber submark = nh.subscribe("/tag_detections_pose", 1, Mark_Handler);  //Marker pose

    //PARA CINEMATICA INVERSA:
    robot_model_loader::RobotModelLoader robot_model_loader("robot1/robot_description");   //--

    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();   //--
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model)); //--
    // kinematic_state->setToDefaultValues(); // --
    const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("Earm");
    group.setPlannerId("RRTkConfigDefault");//PRMstarkConfigDefault---RRTConnectkConfigDefault--RRTkConfigDefault--PRMkConfigDefault--RRTstarkConfigDefault
    group.setGoalTolerance(0.004);//0.004
    group.setGoalOrientationTolerance(0.001);//0.008
    group.setPlanningTime(1.5);
    //JOINTS
    std::vector<double> group_variable_values;
    group.getCurrentState()->copyJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), group_variable_values);
    robot_state::RobotState start_state6(*group.getCurrentState());
    //  group.setStartState(start_state6);
    group_variable_values[0] = 0.0;
    group_variable_values[1] = 0.0;
    group_variable_values[2] = 0.0;
    group_variable_values[3] = 0.0;
    group_variable_values[4] = 0.0;
    group_variable_values[5] = 0.0;
    // group_variable_values[6] = -PI/2;

    group.setJointValueTarget(group_variable_values);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    moveit::planning_interface::MoveItErrorCode success = group.plan(my_plan); //Llamada de Planificacion en move_group

    //PARA ENVIAR TRAYECTORIA   std::cout<<" YAWMARCA "<<yawMark<<" YAWENDEFF "<<yawc<<std::endl;

    control_msgs::FollowJointTrajectoryGoal goaled;
    goaled.trajectory = my_plan.trajectory_.joint_trajectory;
    arm.startTrajectory(goaled);//Inicio de trayectoria en GAZEBO

    sleep(4.0);

    geometry_msgs::Pose currentPose=group.getCurrentPose().pose;
    //  std::cout<<" POSE X: "<<  currentPose.position.x<<" POSE Y: "<<  currentPose.position.y<<" POSE Z: "<<  currentPose.position.z<<" ORI X: "<<  currentPose.orientation.x<<" ORI Y: "<<  currentPose.orientation.y<<" ORI Z: "<<  currentPose.orientation.z<<" ORI W: "<<  currentPose.orientation.z<<std::endl;
    //PRIMER MOVIMIENTO REALIZADO------------------------------------------------------------------------------------------------------

    //AngleAcumulated=(angle1+angle2+angle3+angle4+angle5+angle6+angle7+angle8+angle9+angle10)/10;
    const std::vector<std::string> &joint_names = joint_model_group->getJointModelNames();
    std::vector<double> joint_values;
    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values); //Para kinematica inv

    // group.setStartState(start_state);
    float alturap=0.23;//0.21
    geometry_msgs::Pose target_pose1=currentPose;
    /*target_pose1.orientation.x = -0.7;
    target_pose1.orientation.y = 0.0;
    target_pose1.orientation.w = 0.0;
    target_pose1.orientation.z = 0.0;*/
    target_pose1.position.x = -0.14;//0.1
    target_pose1.position.y = 0.0;
    target_pose1.position.z = alturap;
            //AngleAcumulated=(angle1+angle2+angle3+angle4+angle5+angle6+angle7+angle8+angle9+angle10)/10;lturap;
    bool findKin1=kinematic_state->setFromIK(joint_model_group, target_pose1, 1, 1);


    if (findKin1)  //Cuando exista una solucion kinematica
    {
        std::vector<double> jv2;
        kinematic_state->copyJointGroupPositions(joint_model_group, jv2);

        group.setJointValueTarget(jv2);

         moveit::planning_interface::MoveItErrorCode found_ik1 = group.plan(my_plan);

        if (found_ik1== moveit_msgs::MoveItErrorCodes::SUCCESS)  //Cuando exista una solucion de moveit
        {
            goaled.trajectory = my_plan.trajectory_.joint_trajectory;
            std::cout<<" -----------ConrolPASO: "<<std::endl;
            arm.startTrajectory(goaled);//Inicio de trayectoria en GAZEBO
        }
    }
    // group.setPoseTarget(target_pose1);
    //  success = group.plan(my_plan);
    //PARA ENVIAR TRAYECTORIA A GAZEBO
    //    goaled.trajectory = my_plan.trajectory_.joint_trajectory;
    //  arm.startTrajectory(goaled);//Inicio de trayectoria en GAZEBO
    std::cout<<" -----------ConrolPASO: "<<std::endl;
    double time2=0.3;
    ros::Duration tiempo_traj1(0.0);
    int nump=goaled.trajectory.points.size()-1;
    tiempo_traj1=goaled.trajectory.points[nump].time_from_start;
    time2=tiempo_traj1.toSec();
    std::cout<<"DURACION DE TRAJECTORIA: "<<tiempo_traj1.toSec()<<std::endl;
    usleep(std::abs(500000*(time2)));

    robot_state::RobotState start_state(*group.getCurrentState());
    group.setStartState(start_state);


    //std::cout<<" SEGUNDO PUNTO"<<success<<std::endl;
    /* target_pose1.position.y = 0.01;//0.1
          bool found_ik = k A GAZEBOinematic_state->setFromIK(joint_model_group, target_pose1, 1, 1);
            std::cout<<" PASO POR CICLO1"<<found_ik<<std::endl;
          if (found_ik)  //Cuando exista una solucion kinematica
          {           std::vector<double> jv;
              control_msgs::FollowJointTrajectoryGoal goale;
              kinematic_state->copyJointGroupPositions(joint_model_group, jv);

              goale = arm.makeArmUpTrajectory(jv);
  //  success = group.plan(my_plan);
              arm.startTrajectory(goale);//Inicio de trayectoria en GAZEBO
          }
             sleep(3.0);*/
    int count1=0,mu2=1,mul=1;
    double cont3=0.0,centerx= target_pose1.position.x,centery= target_pose1.position.y;
    double time1=0.3;
    double radio=0.04;
    int puntosC=100;
    int state_ac=0;
    geometry_msgs::Pose target_pose3 =  target_pose1;
    target_pose3.position.z=alturap;
 float xo=0.0,sx=0,sy=0, yo=0.0,x0=0, y0=0,x1=0,x2=0,x3=0,x4=0,y1=0,y2=0,y3=0,x5=0,y4=0,y5=0;
    while (ros::ok())
    {
        if (state==1|| state==20) {     //Cuando el tracking este OK

            state_ac++;
        }
        else
        {
            state_ac=0;
            xo=0.0, yo=0.0;
        }


        group.setGoalTolerance(0.001);//0.004
        group.setGoalOrientationTolerance(0.001);//0.008
        //moveit::planning_interface::MoveGroup::Plan my_plan;
        ros::Duration tiempo_traj(0.0);

        if (state_ac>10){

            //state_ac=0;
            quat IAngleMark =ConvtoAngles(markpose);
            geometry_msgs::Pose currentPose=group.getCurrentPose().pose;

            quat quaternion=toQuaternion(IAngleMark.z,currentPose);//std::cout<<"x"<<currentPose.orientation.x<<" y "<<currentPose.orientation.y<<" z "<<currentPose.orientation.z<<" w "<<currentPose.orientation.w<<std::endl;
            quat poseCurrent =ConvtoAngles(currentPose);
            float xc1 = (markpose.position.x) * sin(poseCurrent.z) +  (markpose.position.y) * cos (poseCurrent.z);
            float yc1 = (markpose.position.x) * cos(poseCurrent.z) -  (markpose.position.y) * sin (poseCurrent.z);

            // std::cout<<"ROLL"<<poseCurrent.x<<" PITCH "<<poseCurrent.y<<" YAW "<<poseCurrent.z<<std::endl;
            target_pose3.orientation.x= quaternion.x;
            target_pose3.orientation.y= quaternion.y;
            target_pose3.orientation.z= quaternion.z;
            target_pose3.orientation.w= quaternion.w;
            float cx,cy,corg=0.0095;
            cx = 0.02*xc1;
            cy = 0.02*yc1;
            if (cx>corg) cx=corg;
             if (cx<-corg) cx=-corg;
             if (cy>corg) cy=corg;
              if (cy<-corg) cy=-corg;
            // quad_pose=target_pose3;
            target_pose3.position.x -= cx;
            target_pose3.position.y += cy;
           // std::cout<<target_pose3.position.x<<" '''''''''' "<<target_pose3.position.y<<std::endl;
            if (  target_pose3.position.x >= 0.18 ) target_pose3.position.x =  0.18;
            if (  target_pose3.position.x <= -0.18) target_pose3.position.x = -0.18;
            if (  target_pose3.position.y >= 0.18 ) target_pose3.position.y =  0.18;
            if (  target_pose3.position.y <= -0.18) target_pose3.position.y = -0.18;



            double cat1, cat2, offx,offy;
            offx=0.0;
            offy=0.0;
            cat1= target_pose3.position.x-offx;
            cat2= target_pose3.position.y-offy;
            double rad=sqrt((cat1*cat1)+(cat2*cat2));
                        double radint=0.12;
                        double theta=0;


            if (rad>=0.18){//Circulo externo
                if(target_pose3.position.x<0){
                    theta=PI+atan(target_pose3.position.y/target_pose3.position.x);
                }
                else{
                    theta=atan(target_pose3.position.y/target_pose3.position.x);
                }
                target_pose3.position.y=0.18*sin(theta);
                target_pose3.position.x=0.18*cos(theta);
            }

            if (rad<=radint){  //Circulo interno
                float xf=target_pose3.position.x, yf=target_pose3.position.y;
               // float xo=currentPose.position.x, yo=currentPose.position.y;

                float dx=1.5*(xf-xo)/1;
                float dy=1.5*(yf-yo)/1, maxC=1.0;

                if ((xf>=0) && (-dx > maxC*xf)){
                    dx=-maxC*xf/1;}
                else{

                    if ((xf<0) && (-dx<maxC*xf))
                    {dx=-maxC*xf/1;}
                }

                if ((yf>=0) && (-dy>maxC*yf)){
                    dy=-maxC*yf/1;
                }
                else
                {
                    if ((yf<0) && (-dy<maxC*yf))
                   {dy=-maxC*yf/1;}
                }
                y5=y4;x5=x4;
                y4=y3;x4=x3;
                y3=y2;x3=x2;
                y2=y1;x2=x1;
                y1=y0;x1=x0;
                y0=dy;x0=dx;

                double dxa=(x0+x1+x2+x3+x4+x5)/6;
                double dya=(y0+y1+y2+y3+y4+y5)/6;


                dx=dxa;
                dy=dya;
                if ((xo>0)&&(xf<0)||(xo<0)&&(xf>0))
                {sx=xf-offx+(dx/30);}
                else
                {sx=xf-offx+dx;}


                if ((yo>0)&&(yf<0)||(yo<0)&&(yf>0))
                {sy=yf-offy+(dy/30);}
                else
                {sy=yf-offy+dy;}

               theta=std::atan2(sy,sx);

               double xf3=radint*cos(theta)+offx;
               double yf3=radint*sin(theta)+offy;

               double corrx=0.6*(xf3-currentPose.position.x);
               double corry=0.6*(yf3-currentPose.position.y);


               if (dx>0&&corrx<0) corrx=0;
               if (dx<0&&corrx>0) corrx=0;
               if (dy>0&&corry<0) corry=0;
               if (dy<0&&corry>0) corry=0;

               xo=target_pose3.position.x;
               yo=target_pose3.position.y;
               target_pose3.position.x += corrx ;

               target_pose3.position.y += corry ;


            }


            control_msgs::FollowJointTrajectoryGoal goaled1;
            //    kinematic_state->setFromIK(joint_model_group, target_pose1, 1, 1);
            // kinematic_state->copyJointGroupPositions(joint_model_group, jvh);
            moveit::planning_interface::MoveItErrorCode found_ik;


            if (5==5)// Para seleccionar entre moveit y envio directo de valores joints
            {                                                                           //Envio directo de los valores de Joints del brazo
                found_ik = kinematic_state->setFromIK(joint_model_group, target_pose3, 1, 1);
                if (found_ik)  //Cuando exista una solucion kinematica
                {           std::vector<double> jv;
                    control_msgs::FollowJointTrajectoryGoal goale;
                    //  std::cout <<found_ik<<"AAAA";
                    kinematic_state->copyJointGroupPositions(joint_model_group, jv);
                    goale = arm.makeArmUpTrajectory(jv);

                    int numpoints6=goale.trajectory.points.size()-1;
                    tiempo_traj=goale.trajectory.points[numpoints6].time_from_start;
                    time1=tiempo_traj.toSec();

                    arm.startTrajectory(goale);//Inicio de trayectoria en GAZEBO

                    std::cout<<"DURACION DE TRAJECTORIA: "<<time1<<std::endl;
                    usleep(1000000*(time1+0.1));

                }
                else
                {
                    std::cout<<"No se encuentra solucion kinematica!"<<std::endl;
                }
                count1++;
            }
            else                                                                    //Uso de moveit para planificar el siguiente punto
            { bool findKin;

                if (5==5)  {//Para escoger entre pose calculation o entre joint calculation
                    //group.setJointValueTarget(jv1);
                    robot_state::RobotState start_state(*group.getCurrentState());
                    group.setStartState(start_state);
                    group.setPoseTarget(target_pose3);
                    found_ik = group.plan(my_plan);
                    if (found_ik== moveit_msgs::MoveItErrorCodes::SUCCESS)  //Cuando exista una solucion
                    {
                        // goaled1.trajectory = trajectoryCart.joint_trajectory;
                        trajectory_msgs::JointTrajectory AuxTraj,Auxtraj1;
                        Auxtraj1 = my_plan.trajectory_.joint_trajectory;
                        AuxTraj = retimeTrajectory(Auxtraj1,3.5);
                        goaled1.trajectory = AuxTraj;
                        int numpoints=goaled1.trajectory.points.size()-1;
                        tiempo_traj=goaled1.trajectory.points[numpoints].time_from_start;
                        time1=tiempo_traj.toSec();
                        // secs = ((double) (end - start)) * 10 / CLOCKS_PER_SEC;
                        // std::cout<<"DURACION DE TRAJECTORIA: "<<time1<<"  ;  "<<secs <<"  ;  "<<(time1-secs-0.05)<<std::endl;

                        arm.startTrajectory(goaled1);//Inicio de trayectoria en GAZEBO

                        std::cout<<"DURACION DE TRAJECTORIA: "<<tiempo_traj.toSec()<<std::endl;
                        usleep(std::abs(1000000*(time1-0.00)));

                        //trajectory_msgs::JointTrajectoryPoint puntoI = goaled.trajectory.points.end();
                        // tiempo_traj  =puntoI.time_from_start.data;
                    }
                    count1++;
                }
                else
                {
                    robot_state::RobotState start_state(*group.getCurrentState());
                    group.setStartState(start_state);
                    findKin=kinematic_state->setFromIK(joint_model_group, target_pose1, 2, 0.1);
                    if(findKin){
                        std::vector<double> jv1;
                        kinematic_state->copyJointGroupPositions(joint_model_group, jv1);
                        group.setJointValueTarget(jv1);

                        found_ik = group.plan(my_plan);
                        if (found_ik== moveit_msgs::MoveItErrorCodes::SUCCESS)  //Cuando exista una solucion kinematica
                        {
                            goaled.trajectory = my_plan.trajectory_.joint_trajectory;
                            tiempo_traj=goaled.trajectory.points[9].time_from_start;
                            time1=tiempo_traj.toSec();

                            // std::cout<<"DURACION DE TRAJECTORIA: "<<tiempo_traj.toSec()<<std::endl;
                            // secs = ((double) (end - start)) * 10 / CLOCKS_PER_SEC;
                            std::cout<<"DURACION DE TRAJECTORIA: "<<time1<<std::endl;

                            arm.startTrajectory(goaled);//Inicio de trayectoria en GAZEBO
                            usleep(1000000*(time1+0.11));

                            //trajectory_msgs::JointTrajectoryPoint puntoI = goaled.trajectory.points.end();
                            // tiempo_traj  =puntoI.time_from_start.data;
                            count1++;
                        }
                    }
                    else{
                        std::cout<<"No se encuentra solucion kinematica!"<<std::endl;
                        count1++;
                    }
                }
            }
        }


        //   usleep(50000);
        loop_rate.sleep();
        ros::spinOnce();
    }
}




