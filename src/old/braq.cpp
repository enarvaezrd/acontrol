#include <ros/ros.h> 
#include <geometry_msgs/Twist.h> 
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf/transform_listener.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <geometry_msgs/PointStamped.h>
#include "sensor_msgs/Range.h"

#include <string>
#include <math.h>
#include <vector>

#include "geometry.h"

class ArtificialPotentialField{
public:
    ArtificialPotentialField(ros::NodeHandle &node) : 
        base_link_("robot2/base_linkq"),
        cmd_pub_(node.advertise<geometry_msgs::Twist>("robot2/cmd_vel", 10)),
        obs_sub_(node.subscribe("/robot2/octomap_full", 10, &ArtificialPotentialField::obstacleCallback, this)),
        goal_sub_(node.subscribe("clicked_point", 10, &ArtificialPotentialField::goalCallback, this)),
        height_sub_(node.subscribe("/robot2/sonar_height", 10, &ArtificialPotentialField::heightCallback, this)),
        pose_sub_(node.subscribe("/robot2/slam_out_pose", 10, &ArtificialPotentialField::poseCallback, this)),
        key_sub_(node.subscribe("turtlebot_teleop/cmd_vel", 10,&ArtificialPotentialField::keyCallback,this))      //keyboard topic
    {
        collision_map_.header.stamp = ros::Time(0);
    }

    void spin(){
        ros::Rate r(3.0);

        //ros::Duration(1).sleep();
        geometry_msgs::Twist cmd,cmda;

        std::vector<double> xvec (1024);
        std::vector<double> yvec (1024);
        std::vector<double> zvec (1024);
         double force = 0.025;

        int mpoint=1024;
        int flag1=0, flag2=0;
        int cont2=0;
        int ti=0;
        int cont=0;
        while(ros::ok()){

            cont++;
                  geometry_msgs::Twist ed_control;
                  if (cont<3)
                  {
                      ed_control.linear.x = key_x;
                      ed_control.linear.y = 0.0;
                      ed_control.linear.z = 0.15;
                      ed_control.angular.y= 0.0;
                      ed_control.angular.x= 0.0;
                      ed_control.angular.z= key_z;
                      cmd_pub_.publish(ed_control);
                 }
                 else{




            if(collision_map_.header.stamp !=ros::Time(0)){//
              //   ROS_ERROR_STREAM("Datos "  <<goal_msg_gl_.point.x <<goal_msg_gl_.point.y<<goal_msg_gl_.point.z);
                std::string map_frame = collision_map_.header.frame_id;
                octomap::OcTree *tree = dynamic_cast<octomap::OcTree*>(octomap_msgs::msgToMap(collision_map_));
                octomap::OcTree::leaf_iterator const end_it = tree->end_leafs();
                double maxv = 0.5;
                double min_dist = 99999999;
                double cntz;
                dmath::Vector3D min_obs;

                ros::Time now = ros::Time::now();
                tf_listener_.waitForTransform(map_frame, base_link_, now, ros::Duration(1)); //Espera hasta que haya una transformacion correcta entre map y base_link
                for(octomap::OcTree::leaf_iterator it = tree->begin_leafs(0); it != end_it; it++){
                   // ROS_ERROR_STREAM("paso2 " );
                    if(it->getOccupancy() < tree->getOccupancyThres()) continue;

                    geometry_msgs::PointStamped p_in, p_out;
                    p_in.header.frame_id = map_frame;
                    p_in.point.x = it.getX();
                    p_in.point.y = it.getY();
                    p_in.point.z = it.getZ();

                    try{
                        p_in.header.stamp = now;
                        tf_listener_.transformPoint(base_link_, p_in, p_out);//transforma el punto p_in a coordenadas de base_link y lo espliega en p_out
                        dmath::Vector3D obs(p_out.point.x, p_out.point.y, p_out.point.z);
                        double dist = magnitude(obs);
                        if(min_dist > dist){
                            min_dist = dist;   //Distancia mas corta observada
                            min_obs = -obs;    //punto mas cercano observado (neg)
                        }
                        
                    }catch(tf::TransformException &ex){
                        ROS_ERROR_STREAM("Exception trying to transform octomap: " << ex.what());
                    }
                }
                dmath::Vector3D Fs;
                Fs += get_potential_force(min_obs, 1.0, 4.0, 1.0, 4.0);

                geometry_msgs::PointStamped goal_msg_lc;
                dmath::Vector3D goal_lc;
                double difx,dify,difz,magt;
                 if(goal_msg_gl_.point.x != 0.0 && goal_msg_gl_.point.y != 0.0 )                     
                      {difx = goal_msg_gl_.point.x - pose_msg_a.pose.position.x;
                      dify = goal_msg_gl_.point.y - pose_msg_a.pose.position.y;
                      difz = goal_msg_gl_.point.z - height_msg_a.range;
                      magt=sqrt((difx*difx)+(dify*dify)+(difz*difz));
                    flag2=1;
                    if(abs(difx)<abs(1.7*difz) && abs(dify)<abs(1.7*difz) && flag1 == 0)
                    {
                        cmda.linear.x = 0.0;
                        cmda.linear.y = 0.0;
                        cmda.linear.z = 0.0;
                        cmd_pub_.publish(cmda);
                        cmd_pub_.publish(cmda);
                        difx = goal_msg_gl_.point.x - pose_msg_a.pose.position.x;
                        dify = goal_msg_gl_.point.y - pose_msg_a.pose.position.y;
                        difz = goal_msg_gl_.point.z - height_msg_a.range;
                        magt=sqrt((difx*difx)+(dify*dify)+(difz*difz));

                        double mag2d = sqrt((difx*difx)+(dify*dify));
                        double radio = (difx/(2*3.14159265359));
                        float div=round(mag2d*30);//numero de divisiones por metro
                        double pend = (dify / difx);//para linea recta entre xy
                        double penda = (difz / difx);//para linea recta


                        flag1=1;

                     int cont1=0;
                        xvec.clear();
                        yvec.clear();
                        zvec.clear();

                     for (int i=0;i < mpoint;i++)
                     {
                         cont1++;
                         double theta = (cont1 * 2 * 3.14159265359) / div;
                         if(cont1 <= div){

                             double valx =  (radio * (theta - sin(theta)));
                             double valy = (pose_msg_a.pose.position.y) + (pend*valx);//linearmente dependiente de x
                             double valz;
                             if(difx<0)
                                  valz = (penda*valx) +  (height_msg_a.range)+ radio*1.4*(1.0 - cos(theta+(3.14159/8)));
                             else
                                  valz = (penda*valx) +  (height_msg_a.range)- radio*1.4*(1.0 - cos(theta+(3.14159/8)));

                                valx = valx+(pose_msg_a.pose.position.x) ;
                             yvec.push_back(valy);
                             xvec.push_back(valx);
                             zvec.push_back(valz);
                          //   ROS_ERROR_STREAM("Vector "  <<valx<<"*"<<valy<<"*"<<valz<<"*****" <<pose_msg_a.pose.position.x << "*"<< pose_msg_a.pose.position.y<< "*"<<height_msg_a.range<< "*"<<goal_msg_gl_.point.x << "*"<<goal_msg_gl_.point.y << "*"<<goal_msg_gl_.point.z);

                         }
                         else
                         {
                             yvec.push_back(goal_msg_gl_.point.y);
                             xvec.push_back(goal_msg_gl_.point.x);
                             zvec.push_back(goal_msg_gl_.point.z);
                         }
                     }

                }
                if (flag1==1)
                     {
                ti++;
                if(ti > 2)   {cont2++; ti=0;}
                   // ROS_ERROR_STREAM("Datos "  <<goal_lc.z<<"*"<<goal_lc.y<<"*"<<goal_lc.x<<"*****" <<xvec[cont2] << "*"<< yvec[cont2]<< "*"<<zvec[cont2] );

                   geometry_msgs::PointStamped goal_part_tf;
                    try{
                           tf_listener_.waitForTransform(goal_msg_gl_.header.frame_id, base_link_, now, ros::Duration(1));
                           goal_msg_gl_.header.stamp = now;
                           goal_part = goal_msg_gl_;
                           goal_part.point.x = xvec[cont2];
                           goal_part.point.y = yvec[cont2];
                           goal_part.point.z = zvec[cont2];
                           tf_listener_.transformPoint(base_link_, goal_part, goal_part_tf);

                           goal_lc = -dmath::Vector3D(goal_part_tf.point.x, goal_part_tf.point.y, goal_part_tf.point.z);
                           // ROS_ERROR_STREAM("Paso1"  << goal_part.point.x<<"**"<<goal_part.point.y<<"**"<< goal_lc.x<<"**"<< goal_lc.y<<"**"<<height_msg_a.range<<"**"<<zvec[cont2]);

                            force=0.026;

                   }

                   catch(tf::TransformException &ex){
                       ROS_ERROR_STREAM("Exception trying to transform goal position: " << ex.what());
                       goal_lc = dmath::Vector3D();
                   }
                }
                else {
                    try{
                        tf_listener_.waitForTransform(goal_msg_gl_.header.frame_id, base_link_, now, ros::Duration(1));
                        goal_msg_gl_.header.stamp = now;
                        tf_listener_.transformPoint(base_link_, goal_msg_gl_, goal_msg_lc);

                        goal_lc = -dmath::Vector3D(goal_msg_lc.point.x, goal_msg_lc.point.y, goal_msg_lc.point.z);
                    //     ROS_ERROR_STREAM("Fuerza111111111111--------------------------- " << goal_lc.x<<"**"<< goal_lc.y<<"**"<<height_msg_a.range);
                         force=0.025;
                    }catch(tf::TransformException &ex){
                        ROS_ERROR_STREAM("Exception trying to transform goal position: " << ex.what());
                        goal_lc = dmath::Vector3D();
                    }

                }

                Fs += get_potential_force(goal_lc, 50, 0, 1, 1);
                
                dmath::Vector3D vel = Fs * force;

                if(vel.x > maxv) vel.x = maxv;
                if(vel.x < -maxv) vel.x = -maxv;
                if(vel.y > maxv) vel.y = maxv;
                if(vel.y < -maxv) vel.y = -maxv;
                if(vel.z > maxv) vel.z = maxv;
                if(vel.z < -maxv) vel.z = -maxv;
                cmd.linear.x = vel.x;
                cmd.linear.y = vel.y;
                if (flag1==1)
                cmd.linear.z = 0.1*(zvec[cont2]-height_msg_a.range);
                else
                    cmd.linear.z =0.0;
      //  ROS_ERROR_STREAM("confirm--------" << vel.x<<"**"<< vel.y<<"**"<<cmd.linear.z);
if (abs(pose_msg_a.pose.position.x -goal_msg_gl_.point.x) <0.001&&abs(pose_msg_a.pose.position.y -goal_msg_gl_.point.y) <0.001)  ROS_ERROR_STREAM("GOAL--------");
                 cmd_pub_.publish(cmd);
            }
                 else{
                     ed_control.linear.x = 1.5*key_x;
                     ed_control.linear.y = 0.0;
                     ed_control.linear.z = 0.0;
                     ed_control.angular.y= 0.0;
                     ed_control.angular.x= 0.0;
                     ed_control.angular.z= 1*key_z;
                     cmd_pub_.publish(ed_control);

                 }
             }
            }

            r.sleep();
            ros::spinOnce();
        }
    }

private:
    dmath::Vector3D get_potential_force(const dmath::Vector3D &dest_lc, double A = 1, double B = 1, double n = 1, double m = 1){
        dmath::Vector3D u = dest_lc;
        u = normalize(u);

        const double d = magnitude(dest_lc);
        double U = 0;
        if(fabs(d) > dmath::tol){
            U = -A/pow(d, n) + B/pow(d, m);
        }
        
        return U * u;
    }

    void obstacleCallback(const octomap_msgs::OctomapPtr &obs_msg){
        collision_map_ = *obs_msg;
    }

    void goalCallback(const geometry_msgs::PointStamped &goal_msg){
        goal_msg_gl_ = goal_msg;
        if(goal_msg_gl_.point.z < 1.5 && goal_msg_gl_.point.z != 0) goal_msg_gl_.point.z = 1.5;
    }

    void heightCallback(const sensor_msgs::Range &height_msg){
        height_msg_a = height_msg;
    }
    void poseCallback(const geometry_msgs::PoseStamped &pose_msg){
        pose_msg_a = pose_msg;
    }
    void keyCallback(const geometry_msgs::Twist &key){
        key_z=key.angular.z;
        key_x=key.linear.x;
    }
    
    octomap_msgs::Octomap collision_map_;
    ros::Publisher cmd_pub_;
    ros::Subscriber obs_sub_, goal_sub_,height_sub_,pose_sub_,key_sub_;

    tf::TransformListener tf_listener_;
    std::string base_link_;
    geometry_msgs::PointStamped goal_msg_gl_ , goal_part;
    sensor_msgs::Range height_msg_a;
    geometry_msgs::PoseStamped pose_msg_a;
   float key_x,key_z;
};

int main(int argc, char *argv[]){

    ros::init(argc, argv, "braq");
   
    ros::NodeHandle node;
    ArtificialPotentialField braq(node);
    braq.spin();
    
    return 0;
}

