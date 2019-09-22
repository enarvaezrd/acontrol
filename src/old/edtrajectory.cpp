#include "edtrajectory.h"

double PI=3.141592654;
namespace katana_tutorials
{

FollowJointTrajectoryClient::FollowJointTrajectoryClient() :
    traj_client_("/robot1/arm_arm_controller/follow_joint_trajectory", true), got_joint_state_(false), spinner_(0)
{
    joint_names_.push_back("Shoulder_Joint");
    joint_names_.push_back("Arm_Joint");
    joint_names_.push_back("Forearm_Joint");
    joint_names_.push_back("Wrist_1_Joint");
    joint_names_.push_back("Wrist_2_Joint");
    joint_names_.push_back("Hand_Joint"); //PARA MICO


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
{
  // When to start the trajectory: 1s from now
  goal.trajectory.header.stamp = ros::Time::now();// + ros::Duration(0.1);

  //std::cout<<goal;
  traj_client_.sendGoal(goal);
}

control_msgs::FollowJointTrajectoryGoal FollowJointTrajectoryClient::makeArmUpTrajectory(std::vector<double> joints_obj)
{
  const size_t NUM_TRAJ_POINTS = 2;
  const size_t NUM_JOINTS = 6;

  // positions after calibration
  std::vector<double> mid_positions(NUM_JOINTS);
  mid_positions[0] = joints_obj[0];
  mid_positions[1] = joints_obj[1];
  mid_positions[2] = joints_obj[2];
  mid_positions[3] = joints_obj[3];
  mid_positions[4] = joints_obj[4];
  mid_positions[5] = joints_obj[5];

  // arm pointing straight up
  std::vector<double> straight_up_positions(NUM_JOINTS);
  straight_up_positions[0] = joints_obj[0];
  straight_up_positions[1] = joints_obj[1];
  straight_up_positions[2] = joints_obj[2];
  straight_up_positions[3] = joints_obj[3];
  straight_up_positions[4] = joints_obj[4];
  straight_up_positions[5] = joints_obj[5];

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
  trajectory.points[ind].time_from_start = ros::Duration(0.00* ind);
  trajectory.points[ind].positions = current_joint_state_;

  // trajectory point:
  ind++;
  trajectory.points[ind].time_from_start = ros::Duration(0.01 * ind);
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

                             float yawc1,yaw1;
            if ( yawc >= 0  )       yawc1 =  yawc - PI ;
            if ( yawc < 0)       yawc1 =  PI + yawc;

            if ( yaw >= 0  )       yaw1 =  -(yaw - PI) ;
            if ( yaw < 0)       yaw1 =  -(PI + yaw);

                     yaw=yawc1-(0.2*yaw1);

                double roll=0;
                         double  pitch=PI;
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
void Depth_Handler(const sensor_msgs::Image::ConstPtr& msg)
{
   DepthImg_CV=cv_bridge::toCvCopy(msg);
}

int LandingFlag;
void LandOrder_Handler(const std_msgs::Int8& msgL)
{
  LandingFlag=msgL.data;
}


using namespace katana_tutorials;

int main(int argc, char** argv)
{
    // Init the ROS node
    ros::init(argc, argv, "follow_joint_trajectory_client");
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);
    katana_tutorials::FollowJointTrajectoryClient arm;
    //MOVEIT GROUP
    moveit::planning_interface::MoveGroupInterface group("arm");
    //PARA CINEMATICA INVERSA:
    robot_model_loader::RobotModelLoader robot_model_loader("robot1/robot_description");   //--

    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();   //--
   robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model)); //--
   kinematic_state->setToDefaultValues(); // --
    const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("arm");
    group.setPlannerId("RRTkConfigDefault");//PRMstarkConfigDefault---RRTConnectkConfigDefault--RRTkConfigDefault--PRMkConfigDefault--RRTstarkConfigDefault

    ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
    ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());

    //JOINTS
    std::vector<double> group_variable_values;
   group.getCurrentState()->copyJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), group_variable_values);
    ros::Subscriber submark = nh.subscribe("/tag_detections_pose", 1, Mark_Handler);  //Marker pose
    ros::Subscriber subDepth = nh.subscribe("/robot1/asus/depth/image_raw", 1, Depth_Handler);  //Marker tracking state
    ros::Subscriber subLandOrder = nh.subscribe("/edtopic/landing_flag", 1, LandOrder_Handler);  //Marker tracking state


    ros::Publisher pub1=nh.advertise<std_msgs::Int8>("/edtopic/tracker_state", 10);
    ros::Publisher pub2=nh.advertise<geometry_msgs::Pose>("/edtopic/quadpose", 10);
    ros::Publisher pub3=nh.advertise<geometry_msgs::Pose>("/edtopic/endeffpose", 10);
     ros::Publisher pub4=nh.advertise<std_msgs::Float64>("/edtopic/temp", 1);//temporal BORRAR
     ros::Publisher pubEEFF=nh.advertise<geometry_msgs::Pose>("/edtopic/eeff", 10);//Para publicar datos del eeff para grafica

    group_variable_values[0] = 0.0;
    group_variable_values[1] = -4.2*PI/6;
    group_variable_values[2] = 4.2*PI/6;
    group_variable_values[3] = 0.0;
    group_variable_values[4] = 0.0;
    group_variable_values[5] = 0.0;

    group.setJointValueTarget(group_variable_values);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    moveit::planning_interface::MoveItErrorCode success = group.plan(my_plan); //Llamada de Planificacion en move_group

        //PARA ENVIAR TRAYECTORIA A GAZEBO
        control_msgs::FollowJointTrajectoryGoal goaled;
        goaled.trajectory = my_plan.trajectory_.joint_trajectory;
        arm.startTrajectory(goaled);//Inicio de trayectoria en GAZEBO

         sleep(5.0);
        group.setGoalTolerance(0.001);
        group.setGoalOrientationTolerance(0.001);

  moveit_msgs::Constraints test_constraints;
     //PATH CONSTRAINTS JOINTtarget_pose3.position.y/target_pose3.position.x
     moveit_msgs::JointConstraint pcm1;
     pcm1.joint_name = "Shoulder_Joint";
     pcm1.position = 0.0;
     pcm1.tolerance_above = PI/6;
     pcm1.tolerance_below = PI/6;
     pcm1.weight = 3.0;
     test_constraints.joint_constraints.push_back(pcm1);
     //PATH CONSTRAINTS JOINT
     moveit_msgs::JointConstraint pcm;
     pcm.joint_name = "Arm_Joint";
     pcm.position = -PI/2;//Entre  1.2---1.6
     pcm.tolerance_above = PI/8;
     pcm.tolerance_below = PI/4;
     pcm.weight = 3.0;
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

     const std::vector<std::string> &joint_names = joint_model_group->getJointModelNames();
     std::vector<double> joint_values;
     kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
     for(std::size_t i = 0; i < joint_names.size(); ++i)
     {
       ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
     }

     robot_state::RobotState start_state(*group.getCurrentState());
     geometry_msgs::Pose start_pose2;
     start_pose2.orientation.x = 0.0;
     start_pose2.orientation.y = 1.0;
     start_pose2.orientation.w = 0.0;
     start_pose2.orientation.z = 0.0;
     start_pose2.position.x = 0.2;//0.1
     start_pose2.position.y = 0.0;
     start_pose2.position.z = 0.8;
     const robot_state::JointModelGroup *joint_model_group1 = start_state.getJointModelGroup(group.getName());
    // start_state.setFromIK(joint_model_group1, start_pose2);
    // group.setStartState(start_state);
     float alturap=1.0;
     geometry_msgs::Pose target_pose1;
     target_pose1.orientation.x = 0.0;
     target_pose1.orientation.y = 1.0;
     target_pose1.orientation.w = 0.0;
     target_pose1.orientation.z = 0.0;
     target_pose1.position.x = 0.25;//0.1
     target_pose1.position.y = 0.15;
     target_pose1.position.z = alturap;
     group.setPathConstraints(test_constraints);
     group.setPoseTarget(target_pose1);
     success = group.plan(my_plan);

     //ROS_INFO("Visualizing plan 3 (constraints) %s",success?"":"FAILED");
         //PARA ENVIAR TRAYECTORIA A GAZEBO
         goaled.trajectory = my_plan.trajectory_.joint_trajectory;
         arm.startTrajectory(goaled);//Inicio de trayectoria en GAZEBO

          sleep(1.0);

 //FUNCION DE EXTRACCION DE LA CINEMATICA INVERSA, PASAR LA POSE Y DA COMO RESULTADO LOS ANGULOS (JOINTS)

          std::vector<double> group_variable_values1 ;
          group.getCurrentState()->copyJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), group_variable_values1);

 //group.setJointValueTarget(group_variable_values1);
std_msgs::Int8 flmess;
int state_ac=0, ac_angle=0, landLock=0,airLock=0, ceC=0,cdiffy=0;
geometry_msgs::Pose target_pose3 =  target_pose1;
//target_pose3.position.z=0.8;
//alturap=0.8;
geometry_msgs::Pose target_pose4 =  target_pose1;
geometry_msgs::Pose quad_pose, endeff_pose;
float correction_z=0;
float deltaD=0.0005;
group.setPathConstraints(test_constraints);
flmess.data=0;
std_msgs::Float64 temped;
temped.data=0.0;
float xo=0.0,sx=0,sy=0, yo=0.0,x0=0, y0=0,x1=0,x2=0,x3=0,x4=0,y1=0,y2=0,y3=0,x5=0,y4=0,y5=0;
 geometry_msgs::Pose lastValid=target_pose1, lastValid1=target_pose1;
while (ros::ok())
{
      clock_t tStart = clock();

    flmess.data=0;
    if (state==1|| state==20) {     //Cuando el tracking este OK
        quat IAngleMark1 =ConvtoAngles(markpose);
        temped.data=IAngleMark1.z;
        geometry_msgs::Pose currentPose1=group.getCurrentPose().pose;
        endeff_pose = currentPose1;
        quat quaternion1=toQuaternion((IAngleMark1.z),currentPose1);//std::cout<<"x"<<currentPose.orientation.x<<" y "<<currentPose.orientation.y<<" z "<<currentPose.orientation.z<<" w "<<currentPose.orientation.w<<std::endl;
        quat poseCurrent1 =ConvtoAngles(currentPose1);


        float xc11 = (markpose.position.x) * sin(poseCurrent1.z) +  (markpose.position.y) * cos (poseCurrent1.z);
        float yc11 = (markpose.position.x) * cos(poseCurrent1.z) -  (markpose.position.y) * sin (poseCurrent1.z);

        // std::cout<<"ROLLMARCA"<<posequat.x<<" PITCH "<<posequat.y<<" YAWMARCA "<<posequat.z<<std::endl;
        quad_pose=target_pose3;
        quad_pose.orientation.x= quaternion1.x;
        quad_pose.orientation.y= quaternion1.y;
        quad_pose.orientation.z= quaternion1.z;
        quad_pose.orientation.w= quaternion1.w;
        quad_pose.position.x -= xc11;
        quad_pose.position.y += yc11;

       pub2.publish(quad_pose); //Para controlar el Quad sin entrar en el control del brazo
       pub3.publish(endeff_pose); //Para enviar informacion al nodo de acontrol,

       flmess.data=1;
            state_ac++;
    }
    else {
            state_ac=0;ac_angle=0;    //Para disminuir la altura en caso de que se pierda el tracking
            target_pose3.position.z -=0.02;
             if (target_pose3.position.z<=alturap) {target_pose3.position.z=alturap; }
            bool found_ik = kinematic_state->setFromIK(joint_model_group, target_pose3, 1, 1);
            //PARA ENVIAR TRAYECTORIA A GAZEBO
               std::vector<double> jv;
            control_msgs::FollowJointTrajectoryGoal goale;
            //  std::cout <<found_ik<<"AAAA";

            kinematic_state->copyJointGroupPositions(joint_model_group, jv);
            goale = arm.makeArmUpTrajectory(jv);
            arm.startTrajectory(goale);//Inicio de trayectoria en GAZEBO
             usleep(100000);flmess.data=0;
    }

    if (state_ac>=20){  //Cuando el Quad se haya detectado varias veces se continua al control del brazo
        //flmess.data=1;

        ac_angle++;
        if (ac_angle>0){ //cada cierto numero de iteraciones

            ac_angle=0;
            //state_ac=0;
            quat IAngleMark =ConvtoAngles(markpose);
            geometry_msgs::Pose currentPose=group.getCurrentPose().pose;
            //AngleAcumulated=(angle1+angle2+angle3+angle4+angle5+angle6+angle7+angle8+angle9+angle10)/10;

            quat quaternion=toQuaternion(IAngleMark.z,currentPose);//std::cout<<"x"<<currentPose.orientation.x<<" y "<<currentPose.orientation.y<<" z "<<currentPose.orientation.z<<" w "<<currentPose.orientation.w<<std::endl;
            quat poseCurrent =ConvtoAngles(currentPose);
            float xc1 = (markpose.position.x) * sin(poseCurrent.z) +  (markpose.position.y) * cos (poseCurrent.z);
            float yc1 = (markpose.position.x) * cos(poseCurrent.z) -  (markpose.position.y) * sin (poseCurrent.z);

            // std::cout<<"ROLLMARCA"<<posequat.x<<" PITCH "<<posequat.y<<" YAWMARCA "<<posequat.z<<std::endl;
            target_pose3.orientation.x= quaternion.x;
            target_pose3.orientation.y= quaternion.y;
            target_pose3.orientation.z= quaternion.z;
            target_pose3.orientation.w= quaternion.w;
            float cx,cy,corg=0.0095;
            cx = 0.2*xc1;
            cy = 0.2*yc1;
            if (cx>corg) cx=corg;
             if (cx<-corg) cx=-corg;
             if (cy>corg) cy=corg;
              if (cy<-corg) cy=-corg;
            // quad_pose=target_pose3;
            target_pose3.position.x -= cx;
            target_pose3.position.y += cy;
            if (  target_pose3.position.x >= 0.35) target_pose3.position.x = 0.35;
            if (  target_pose3.position.x <= -0.35) target_pose3.position.x = -0.35;


            double cat1, cat2, offx,offy;
            offx=0.0;
            offy=0.0;
            cat1= target_pose3.position.x-offx;
            cat2= target_pose3.position.y-offy;
            double rad=sqrt((cat1*cat1)+(cat2*cat2));
            double radint=0.2;
            double theta=0, thetac=0;
            std::cout<<"  **** RADIO ***" << rad<<std::endl;

            if (rad <= radint){  //Circulo interno

                std::cout<<" ----------------ENTRADA--------------------- "<<std::endl;

               //std::cout<<" ----------------ENTRADA--------------------- Deseado X : " <<target_pose3.position.x <<" Y "<<target_pose3.position.y<<"  Z   "<<target_pose3.position.z<<" THETA "<<theta<<" yaw "<<AnglesAux.z<<" pitch "<<AnglesAux.y<<" roll "<<AnglesAux.x<< std::endl;

                if (5==1){//selector de tipos
                theta=std::atan2(cat2,cat1);

//                if(target_pose3.position.x<0){
//                    theta=PI+atan(target_pose3.position.y/target_pose3.position.x);
//                }
//                else{
//                    theta=atan(target_pose3.position.y/target_pose3.position.x);
//                }

               // quat  AnglesAux = ConvtoAngles(target_pose3);
               // quat  QAnglesAux;

                double xf3=radint*cos(theta)+offx;
                double yf3=radint*sin(theta)+offy;

                double corrx=1*(xf3-currentPose.position.x);
                double corry=1*(yf3-currentPose.position.y);

                target_pose3.position.x += corrx ;

                target_pose3.position.y += corry ;
                }


                else{
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




                std::cout<<"****DX***" << sx<<"*****DY*******"<<sy<<"*******";


               theta=std::atan2(sy,sx);

               double xf3=radint*cos(theta)+offx;
               double yf3=radint*sin(theta)+offy;

               double corrx=0.5*(xf3-currentPose.position.x);
               double corry=0.5*(yf3-currentPose.position.y);


               if (dx>0&&corrx<0) corrx=0;
               if (dx<0&&corrx>0) corrx=0;
               if (dy>0&&corry<0) corry=0;
               if (dy<0&&corry>0) corry=0;

               xo=target_pose3.position.x;
               yo=target_pose3.position.y;
               target_pose3.position.x += corrx ;

               target_pose3.position.y += corry ;

                }

            }
            else {
                y5=0;x5=0;
                y4=0;x4=0;
                y3=0;x3=0;
                y2=0;x2=0;
                y1=0;x1=0;
                y0=0;x0=0;
            }

           currentPose=group.getCurrentPose().pose;
           if (state_ac>200){
                pubEEFF.publish(currentPose);
           }
            // quat  AnglesAux1 =ConvtoAngles(target_pose3);
          //    quat  AnglesAuxC1 =ConvtoAngles(currentPose);

          //  std::cout<<" POSICIONDESEADA X"<<target_pose3.position.x<<" Y  "<<target_pose3.position.y<<"Yaw"<< AnglesAux1.z<<"Roll"<< AnglesAux1.x<<"Pitch"<<AnglesAux1.y<<std::endl;
         //   std::cout <<"*///////Current EEFF POSICION***** X  "<< currentPose.position.x<<"** Y ****"<<currentPose.position.y<<"Yaw"<< AnglesAuxC1.z<<"Roll"<< AnglesAuxC1.x<<"Pitch"<<AnglesAuxC1.y<<std::endl;


            bool found_ik = kinematic_state->setFromIK(joint_model_group, target_pose3, 1, 1);
             std::cout<<"found IK??    " <<found_ik<<std::endl;
            flmess.data=2;
            if (found_ik)  //Cuando exista una solucion kinematica
            {
                state_ac++;

                // flmess.data=markpose.position.z;//poseCurrent.z;
                ac_angle=2;

                //PARA ENVIAR TRAYECTORIA A GAZEBO
                std::vector<double> jv;
                control_msgs::FollowJointTrajectoryGoal goale;

                kinematic_state->copyJointGroupPositions(joint_model_group, jv);
                goale = arm.makeArmUpTrajectory(jv);
                arm.startTrajectory(goale);//Inicio de trayectoria en GAZEBO


                if (LandingFlag==1 || landLock==1) {
                    landLock=1;
                    airLock=0;
                     correction_z=(0.0085*markpose.position.z);
                        float taux1=round((1.01+(0.2*(markpose.position.z-0.032)))*50000);

                    int time_sleep = (taux1);
                    std::cout<<" TIEMPO DE ESPERA  "<<time_sleep<<std::endl;
                    usleep(time_sleep);
                    if (markpose.position.z <= 0.0375) //Altura de aterrizaje0.0395
                    {
                        correction_z = 0.0;
                        //landLock=0;
                    }
                    target_pose3.position.z += correction_z ;
                }

                    if (LandingFlag==2 || airLock==1){
                        landLock=0;
                        airLock=1;
                        target_pose3.position.z -= 0.02;
                        if (target_pose3.position.z<=alturap) {target_pose3.position.z=alturap; airLock=0; }
                        usleep(100000);
                    }
                    lastValid1=lastValid;
                    lastValid=group.getCurrentPose().pose;
                  if (LandingFlag==0 && airLock==0 && landLock==0) {  usleep(50000); }
                  if (LandingFlag==1 && airLock==0 && landLock==0) {  usleep(40000); }
            }
            else
            {
                flmess.data=3;  //Nohay solucion kinematica, reiniciar grupo para posibles atascos
                if (state==1) {robot_model_loader::RobotModelLoader robot_model_loader("robot1/robot_description");   //--Reinicio

                    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
                    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));

                    target_pose3 = lastValid; //Poner algo que desatasque el brazo, en este punto deberia tender a buscar una posicion alcanzable
                }
            }
        }
    }
    usleep(6000);pub1.publish(flmess);pub4.publish(temped);
   // loop_rate.sleep();
    ros::spinOnce();
    printf("Tiempo de ejecucion: %.4fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);
}
 }
