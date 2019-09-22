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
        base_link_("base_linkq"),
        cmd_pub_(node.advertise<geometry_msgs::Twist>("/cmd_vel", 10)),
        obs_sub_(node.subscribe("octomap_full", 10, &ArtificialPotentialField::obstacleCallback, this)),
        goal_sub_(node.subscribe("clicked_point", 10, &ArtificialPotentialField::goalCallback, this)),
        height_sub_(node.subscribe("sonar_height", 10, &ArtificialPotentialField::heightCallback, this)),
        pose_sub_(node.subscribe("slam_out_pose", 10, &ArtificialPotentialField::poseCallback, this))
    {
        collision_map_.header.stamp = ros::Time(0);
    }

    void spin(){
        ros::Rate r(10);
        
        //ros::Duration(1).sleep();
        geometry_msgs::Twist cmd;

        std::vector<double> xvec (1024);
        std::vector<double> yvec (1024);
        std::vector<double> zvec (1024);
        const double force = 0.025;

        int mpoint=1024;
        int flag1=0;
        int cont2=0;
        int ti=0;
        int cont=0;
        while(ros::ok()){

            cont++;
                  geometry_msgs::Twist ed_control;
                  if (cont<10)
                  {
                      ed_control.linear.x = 0.0;
                      ed_control.linear.y = 0.0;
                      ed_control.linear.z = 0.35;
                      ed_control.angular.y= 0.0;
                      ed_control.angular.x= 0.0;
                      ed_control.angular.z= 0.0;
                      cmd_pub_.publish(ed_control);
                 }
                 else{




            if(collision_map_.header.stamp !=ros::Time(0)){//
              //   ROS_ERROR_STREAM("Datos "  <<goal_msg_gl_.point.x <<goal_msg_gl_.point.y<<goal_msg_gl_.point.z);
                std::string map_frame = collision_map_.header.frame_id;
                octomap::OcTree *tree = dynamic_cast<octomap::OcTree*>(octomap_msgs::msgToMap(collision_map_));
                octomap::OcTree::leaf_iterator const end_it = tree->end_leafs();
                double maxv = 1.0;
                double min_dist = 99999999;
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
                dmath::Vector3D goal_alt;

                try{

                    tf_listener_.waitForTransform(goal_msg_gl_.header.frame_id, base_link_, now, ros::Duration(1));
                    goal_msg_gl_.header.stamp = now;
                    tf_listener_.transformPoint(base_link_, goal_msg_gl_, goal_msg_lc);

                    goal_lc = -dmath::Vector3D(goal_msg_lc.point.x, goal_msg_lc.point.y, goal_msg_lc.point.z);

                    if(goal_msg_gl_.point.x != 0.0 && goal_msg_gl_.point.y != 0.0 && flag1 == 0)
                    {
                        flag1=1;
                    const double mag1 = magnitude(goal_lc);
                     float div=round(mag1*1000);//numero de divisiones por metro
                     int cont1=0;
                     double pend = abs(goal_lc.y / goal_lc.x);
                     double diffz = height_msg_a.range - goal_lc.z ;

                        xvec.clear();
                        yvec.clear();
                        zvec.clear();
                     double radio = abs(goal_lc.x/(2*3.14159));
                     for (int i=0;i < mpoint;i++)
                     {
                         cont1++;
                         double theta = (cont1 * 2 * 3.14159) / div;
                         if(cont1 < div){

                             double valx = (radio * theta) - sin(theta);
                             double valz = radio - cos(theta);
                             double valy=pend*valx;
                             yvec.push_back(valy);
                             xvec.push_back(valx);
                             zvec.push_back(valz);
                         }
                         else
                         {
                             yvec.push_back(goal_lc.y);
                             xvec.push_back(goal_lc.x);
                             zvec.push_back(goal_lc.z);
                         }
                     }
                    }


                }catch(tf::TransformException &ex){
                    ROS_ERROR_STREAM("Exception trying to transform goal position: " << ex.what());
                    goal_lc = dmath::Vector3D();
                }
                ti++;
                if(ti > 10)   {cont2++; ti=0;}
                   // ROS_ERROR_STREAM("Datos "  <<goal_lc.z<<"*"<<goal_lc.y<<"*"<<goal_lc.x<<"*****" <<xvec[cont2] << "*"<< yvec[cont2]<< "*"<<zvec[cont2] );
                    ROS_ERROR_STREAM("Datos "  << height_msg_a.range<<"-**"<<goal_lc.z<<"-**"<< goal_msg_gl_.point.z);
                 goal_alt= -dmath::Vector3D(xvec[cont2], yvec[cont2], zvec[cont2]);

                Fs += get_potential_force(goal_alt, 50, 0, 1, 1);
                
                dmath::Vector3D vel = Fs * force;

                if(vel.x > maxv) vel.x = maxv;
                if(vel.x < -maxv) vel.x = -maxv;
                if(vel.y > maxv) vel.y = maxv;
                if(vel.y < -maxv) vel.y = -maxv;
                if(vel.z > maxv) vel.z = maxv;
                if(vel.z < -maxv) vel.z = -maxv;
                cmd.linear.x = vel.x;
                cmd.linear.y = vel.y;
                cmd.linear.z = vel.z;
                
                 cmd_pub_.publish(cmd);
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
        if(goal_msg_gl_.point.z < 1.0 && goal_msg_gl_.point.z != 0) goal_msg_gl_.point.z = 1.0;
    }

    void heightCallback(const sensor_msgs::Range &height_msg){
        height_msg_a = height_msg;
    }
    void heightCallback(const sensor_msgs::Range &pose_msg){
        pose_msg_a = pose_msg;
    }
    
    octomap_msgs::Octomap collision_map_;
    ros::Publisher cmd_pub_;
    ros::Subscriber obs_sub_, goal_sub_,height_sub_,pose_sub_;

    tf::TransformListener tf_listener_;
    std::string base_link_;
    geometry_msgs::PointStamped goal_msg_gl_;
    sensor_msgs::Range height_msg_a;
    geometry_msgs::PoseStamped pose_msg_a;
};

int main(int argc, char *argv[]){

    ros::init(argc, argv, "braq");
   
    ros::NodeHandle node;
    ArtificialPotentialField braq(node);
    braq.spin();
    
    return 0;
}

