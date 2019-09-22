#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include <geometry_msgs/Twist.h>

float key_x,key_z;
void Key_Handler(const geometry_msgs::Twist& key)
{
    key_z=key.angular.z;
    key_x=key.linear.x;
}



int main(int argc, char **argv)
{
int cont=0;
  ros::init(argc, argv, "talker");
ros::NodeHandle n;
  ros::Subscriber sub5 = n.subscribe("/robot2/turtlebot_teleop/cmd_vel", 10, Key_Handler);        //keyboard topic


  ros::Publisher pub1=n.advertise<geometry_msgs::Twist>("/robot2/cmd_vel", 10);

  ros::Rate loop_rate(10);


  int count = 0;
  while (ros::ok())
  {cont++;
              geometry_msgs::Twist ed_control;
      if (cont<10)
      {
          ed_control.linear.x = key_x;
          ed_control.linear.y = 0.0;
          ed_control.linear.z = 0.3;
          ed_control.angular.y= 0.0;
          ed_control.angular.x= 0.0;
          ed_control.angular.z= key_z;
     }
      else
      {
          ed_control.linear.x = key_x;
          ed_control.angular.z= key_z;
	  ed_control.linear.y = 0.0;
          ed_control.linear.z = 0.1;
          ed_control.angular.y= 0.0;
          ed_control.angular.x= 0.0;
      }
    pub1.publish(ed_control);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }
  return 0;
}
