#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Range.h>
#include <hector_uav_msgs/Altimeter.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int8.h>
#include<iostream>

/*
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <ar_track_alvar_msgs/AlvarMarker.h>

float mx=0,my=0;

void Marker(const ar_track_alvar_msgs::AlvarMarkers& mark)
{
96210] [56.811000] Stopping all controllers...
[robot2/controller_spawnerq-17] e
    if ((mark.markers[0].pose.pose.position.x+0.2)!=0.0 && (mark.markers[0].pose.pose.position.y+0.2)!=0.0)
    {
        mx=abs(600*(mark.markers[0].pose.pose.position.x+0.2))/0.4;
        my=abs(600*(mark.markers[0].pose.pose.position.y+0.2))/0.4;
    }
}

void imageCB(const sensor_msgs::ImageConstPtr& msg)
{
    IplImage cvImageRef, *cvImage;
    CvSize size;
    const sensor_msgs::Image img = *msg;
    cv_bridge::CvImagePtr image = cv_bridge::toCvCopy(msg);
    cvImageRef = IplImage(image->image);
    cvImage = &cvImageRef;

    size = cvGetSize(cvImage);

    if (mx<581 && mx>0 && my<581 && my>0)
    {
        cvRectangle(cvImage, cvPoint((int)(mx), (int)(my)), cvPoint((int)(mx+10), (int)(my+10)), CV_RGB(0, 0, 0));
        mx=0,my=0;
    }

    cvShowImage("Second Robot - External View", cvImage);

    cvWaitKey(20);
}
ros::Subscriber  subm;
subm= n.subscribe("/robot1/ar_pose_marker", 1, Marker);
ros::Subscriber image_subscriber_ = n.subscribe("/robot1/camerao/image_raw", 1, imageCB);*/


float key_x,key_az,key_y,key_z,altitude,range,GPS,landing_flag,height;


void Key_Handler(const geometry_msgs::Twist& key)
{
    key_az=0.5*key.angular.z;
    key_x=0.5*key.linear.x;
    key_y=0.5*key.linear.y;
    key_z=0.5*key.linear.z;
    landing_flag = key.angular.y;
}
void alt_Handler(const hector_uav_msgs::Altimeter& alt)
{
    altitude=alt.altitude;
}
void sonar_Handler(const sensor_msgs::Range& rng)
{
    range=rng.range;
}
void GPS_Handler(const sensor_msgs::NavSatFix& gp)
{
    GPS=gp.altitude;
}
struct quat {
    float x,y,z,w;
}quaternion;

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

void Mark_Handler(const geometry_msgs::PoseStamped& ma)
{
    markpose=ma.pose;
}
int EdTrackingState;
void EdTrackerState_Handler(const std_msgs::Int8& ma1)
{
    EdTrackingState=ma1.data;
}
bool stateGrasp=false;
void StGrasp_Handler(const std_msgs::Bool& state)
{
    stateGrasp=state.data;
}
geometry_msgs::Pose quad_pose;
void QPose_Handler(const geometry_msgs::Pose& qp)
{
    quad_pose=qp;
}
geometry_msgs::Pose eeff_pose;
void EEffPose_Handler(const geometry_msgs::Pose& eep)
{
    eeff_pose=eep;
}
float summit_vel;
void SummitVel_Handler(const geometry_msgs::Twist& sum)
{
    summit_vel= abs(sum.linear.x) ;
}
int state;
void Status_Handler(const std_msgs::Int8& st)
{
    state=0;
    state=st.data;
}

int main(int argc, char **argv)
{
int cont=0,cont1=0;
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  geometry_msgs::Twist ed_control, sum_control;
  ed_control.linear.x = 0.0;
  ed_control.linear.y = 0.0;
  ed_control.linear.z = 0.4;
  ed_control.angular.y= 0.0;
  ed_control.angular.x= 0.0;
  ed_control.angular.z= 0.0;
  ros::Rate loop_rate(3.0);
  ros::Publisher pub1=n.advertise<geometry_msgs::Twist>("/robot2/cmd_vel", 1);//robot2/cmd_vel
  ros::Publisher pub2=n.advertise<std_msgs::Int8>("/edtopic/landing_flag", 1);//robot2/cmd_vel
//  ros::Publisher pub3=n.advertise<geometry_msgs::Twist>("/summit_xl_control/cmd_vel", 1);//robot2/cmd_vel
  pub1.publish(ed_control); ros::spinOnce(); loop_rate.sleep();
  pub1.publish(ed_control);ros::spinOnce(); loop_rate.sleep();
  ros::Subscriber sub5 =   n.subscribe("/turtlebot_teleop/cmd_vel", 1, Key_Handler);         //keyboard topic
  ros::Subscriber subb =   n.subscribe("/robot2/altimeter", 1, alt_Handler);                 //keyboard topic
  ros::Subscriber subs =   n.subscribe("/robot2/sonar_height", 1, sonar_Handler);            //keyboard topic
  ros::Subscriber subgps = n.subscribe("/robot2/fix", 1, GPS_Handler);                       //keyboard topic
  ros::Subscriber submark = n.subscribe("/visp_auto_tracker/object_position", 1, Mark_Handler); //Marker pose
  ros::Subscriber submark1 = n.subscribe("/edtopic/tracker_state", 1, EdTrackerState_Handler);  //Marker pose
  ros::Subscriber subTrackState = n.subscribe("/visp_auto_tracker/status", 1, Status_Handler);  //Marker tracking state
  ros::Subscriber subgraspstate = n.subscribe("/robot1/grasping", 10, StGrasp_Handler);  //Marker pose
  ros::Subscriber subposequad = n.subscribe("/edtopic/quadpose", 10, QPose_Handler);  //Marker pose
  ros::Subscriber subposearm = n.subscribe("/edtopic/endeffpose", 10, EEffPose_Handler);  //Marker pose
  ros::Subscriber subvelsumm= n.subscribe("/summit_xl_control/cmd_vel", 10, SummitVel_Handler);  //Marker pose

  float error=0.0,sv1=0,sv2=0,sv3=0,sv4=0,sv5=0,svp=0,contvel=0;
  int count = 0, grflag=0;
  float PI=3.141592654;
  std_msgs::Int8 landing_order;

  while (ros::ok())
  {
      landing_order.data= int (landing_flag);


      quat q3= ConvtoAngles(markpose);
      quat poseCurrent =ConvtoAngles(quad_pose);
      float xland =quad_pose.position.x-0.05;  //Traslacion
      float yland =quad_pose.position.y-0.24;
        float quad_angle;

        if (poseCurrent.z > 0.0)   {quad_angle =  3.141592654 - poseCurrent.z ; }
        if (poseCurrent.z <= 0.0)  {quad_angle = -3.141592654 - poseCurrent.z ; }


       quad_angle = 0.0-poseCurrent.z ;
        quad_angle = fmod(quad_angle,(PI));
      float yc1 = (xland) * sin(quad_angle) + (yland) * cos (quad_angle); //Rotacion, Punto de aterrizaje visto desde el Quad
      float xc1 = ((xland) * cos(quad_angle) - (yland) * sin (quad_angle));

      float xerr = xc1; //Cantidad a corregir
      float yerr = yc1;
float le1=0.4;


      float diff = sqrt((xland*xland)+(yland*yland));  //Distancia euclidea entre el Quad y el punto de aterrizaje

      float Kp = 0;

      if (state != 3)
      {
          Kp = 0.65;//se mueve sin que haya movimiento del brazo
          sv5=0;
          sv4=0;
          sv3=0;
          sv2=0;
          sv1=0;
          svp=0;contvel=0;
          if(diff > 2.0 || EdTrackingState == 0)
          {Kp = 0.0; }
      }
      else   {
          contvel++;
          if (contvel > 5) {
          sv5=sv4;
          sv4=sv3;
          sv3=sv2;
          sv2=sv1;
          sv1=summit_vel;
          svp=(sv1+sv2+sv3+sv4+sv5)/5;
          Kp = 0.65 + svp*1.4;// Constante de control de posicion del Quad
          }
        else   {
              svp=0;Kp = 0.5;
                if (contvel > 1) Kp = 0.6;

                if (xerr>le1) xerr=le1;
                if (xerr<-le1) xerr=-le1;
                if (yerr>le1) yerr=le1;
                if (yerr<-le1) yerr=-le1;

              }
          if(diff > 2.0 || EdTrackingState == 0){Kp = 0.0; }
          if( EdTrackingState != 2)//en caso de que el brazo se atasque
          {Kp = Kp; }
      }


        float heighmark;
     // std::cout<<"//EdTrackingState: "<<EdTrackingState<<" Dff: "<<diff<<std::endl;
      if (state!=3){
        height= GPS+0.075;
        cont1=0;
      }
      else{
          cont1++;
          if (cont1>4){
          if((1.55*markpose.position.z) >= 0.5) { heighmark = 0.5/1.55; }
          else
           {heighmark =markpose.position.z;}
          height= eeff_pose.position.z + (1.55*heighmark) +0.2;
          }
          else
              {height= GPS+0.075;}
      }

      //float yawcalculado=yawmarca+yawrobot;//pitch+PI;//Offset
      //std::cout<<"//Yaw Marca: "<<quad_angle<<" Error X: "<<xerr<<" Error Y: "<<yerr<<" KP: "<<Kp<<" Diferencia : "<<diff<<" Xland : "<<xland<<" Yland : "<<yland<<" State : "<<EdTrackingState<<std::endl;
      cont++;
      if (cont<18)
      {
          ed_control.linear.x =  key_x;
          ed_control.linear.y =  key_y;
          ed_control.linear.z = 0.2;
          ed_control.angular.y= 0.0;
          ed_control.angular.x= 0.0;
          ed_control.angular.z= key_az;
      }
      else
      {
          //std::cout<<altitude<<"AAAAAAA"<<range<<"GGGGGGGGGGG"<<GPS<<std::endl;
          error=1.33-height;//1.33
          if (error>1) error=1;
          if (error<-1) error=-1;
//Kp=0;
          ed_control.linear.x = 1.1*Kp * xerr ;//+ key_x;
          ed_control.linear.y = Kp * yerr ;//+ key_y;
          ed_control.linear.z = key_z+(error*0.25);
          ed_control.angular.y= 0.0;
          ed_control.angular.x= 0.0;
          ed_control.angular.z= 0.0;//key_az;
          if (cont<15) ed_control.linear.z = 0.005;
      }

      if (stateGrasp == true || grflag!=0) {
          ed_control.linear.x = 0.0;
          ed_control.linear.y = 0.0;
          ed_control.linear.z = -0.7;
          ed_control.angular.y= 0.0;
          ed_control.angular.x= 0.0;
          ed_control.angular.z= 0.0;
          grflag=1;
      }

      sum_control.linear.x = key_x;
      sum_control.linear.y = 0.0;
      sum_control.linear.z = 0.0;
      sum_control.angular.y= 0.0;
      sum_control.angular.x= 0.0;
      sum_control.angular.z= key_y*4;

      if (key_az != 0.0  )  {

          sum_control.linear.x = (key_az*key_az)*0.4;
          sum_control.angular.z= 0.5*key_az;

      }
      pub1.publish(ed_control);
      pub2.publish(landing_order);
    //  pub3.publish(sum_control);
      ros::spinOnce();

      loop_rate.sleep();
      ++count;
  }
  return 0;
}
