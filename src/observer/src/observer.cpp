#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <ar_track_alvar_msgs/AlvarMarker.h>
#include <stdio.h>
#include <vector>
#include <math.h>
#include <iostream>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h> 
#include <tf/tf.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include "std_msgs/Float32MultiArray.h"
#include <unsupported/Eigen/MatrixFunctions>

using namespace std;
//float n[6][4] = {{2,1,1.5,1},{2.3,-0.6,0.1,1},{2.5,-0.4,1.1,1},{1.5,-1,0.6,1},{1.5,0,0.1,1},{2.5,-1,1.1,1}}; //
float n[7][4] = {{0,0,0,1},{2,0,0.2,1},{2.5,1.2,0.3,1},{2.5,-1.1,0.25,1},{1.5,-0.8,0.2,1},{4.3,-0.5,0.3,1},{3.4,1,0.3,1}};
int f = 0;
Eigen::Matrix4d SE_3,SE_QB;
double lvx,lvy,lvz,avx,avy,avz,x,y,z,qx,qy,qz,qw;
std_msgs::Float32MultiArray dat;
int flag;
geometry_msgs::PoseStamped pose;



Eigen::Matrix4d inverse (Eigen::Matrix4d T)
      {
       double det;
       Eigen::Matrix4d invT;
       Eigen::Matrix4d inv;
       inv(0,0) = T(1,1)  * T(2,2) * T(3,3) - 
             T(1,1)  * T(2,3) * T(3,2) - 
             T(2,1)  * T(1,2)  * T(3,3) + 
             T(2,1)  * T(1,3)  * T(3,2) +
             T(3,1) * T(1,2)  * T(2,3) - 
             T(3,1) * T(1,3)  * T(2,2);

    inv(1,0) = -T(1,0)  * T(2,2) * T(3,3) + 
              T(1,0)  * T(2,3) * T(3,2) + 
              T(2,0)  * T(1,2)  * T(3,3) - 
              T(2,0)  * T(1,3)  * T(3,2) - 
              T(3,0) * T(1,2)  * T(2,3) + 
              T(3,0) * T(1,3)  * T(2,2);

    inv(2,0) = T(1,0)  * T(2,1) * T(3,3) - 
             T(1,0)  * T(2,3) * T(3,1) - 
             T(2,0)  * T(1,1) * T(3,3) + 
             T(2,0)  * T(1,3) * T(3,1) + 
             T(3,0) * T(1,1) * T(2,3) - 
             T(3,0) * T(1,3) * T(2,1);

    inv(3,0) = -T(1,0)  * T(2,1) * T(3,2) + 
               T(1,0)  * T(2,2) * T(3,1) +
               T(2,0)  * T(1,1) * T(3,2) - 
               T(2,0)  * T(1,2) * T(3,1) - 
               T(3,0) * T(1,1) * T(2,2) + 
               T(3,0) * T(1,2) * T(2,1);

    inv(0,1) = -T(0,1)  * T(2,2) * T(3,3) + 
              T(0,1)  * T(2,3) * T(3,2) + 
              T(2,1)  * T(0,2) * T(3,3) - 
              T(2,1)  * T(0,3) * T(3,2) - 
              T(3,1) * T(0,2) * T(2,3) + 
              T(3,1) * T(0,3) * T(2,2);

    inv(1,1) = T(0,0)  * T(2,2) * T(3,3) - 
             T(0,0)  * T(2,3) * T(3,2) - 
             T(2,0)  * T(0,2) * T(3,3) + 
             T(2,0)  * T(0,3) * T(3,2) + 
             T(3,0) * T(0,2) * T(2,3) - 
             T(3,0) * T(0,3) * T(2,2);

    inv(2,1) = -T(0,0)  * T(2,1) * T(3,3) + 
              T(0,0)  * T(2,3) * T(3,1) + 
              T(2,0)  * T(0,1) * T(3,3) - 
              T(2,0)  * T(0,3) * T(3,1) - 
              T(3,0) * T(0,1) * T(2,3) + 
              T(3,0) * T(0,3) * T(2,1);

    inv(3,1) = T(0,0)  * T(2,1) * T(3,2) - 
              T(0,0)  * T(2,2) * T(3,1) - 
              T(2,0)  * T(0,1) * T(3,2) + 
              T(2,0)  * T(0,2) * T(3,1) + 
              T(3,0) * T(0,1) * T(2,2) - 
              T(3,0) * T(0,2) * T(2,1);

    inv(0,2) = T(0,1)  * T(1,2) * T(3,3) - 
              T(0,1)  * T(1,3) * T(3,2) - 
              T(1,1)  * T(0,2) * T(3,3) + 
              T(1,1)  * T(0,3) * T(3,2) + 
              T(3,1) * T(0,2) * T(1,3) - 
              T(3,1) * T(0,3) * T(1,2);

    inv(1,2) = -T(0,0)  * T(1,2) * T(3,3) + 
              T(0,0)  * T(1,3) * T(3,2) + 
              T(1,0)  * T(0,2) * T(3,3) - 
              T(1,0)  * T(0,3) * T(3,2) - 
              T(3,0) * T(0,2) * T(1,3) + 
              T(3,0) * T(0,3) * T(1,2);

    inv(2,2) = T(0,0)  * T(1,1) * T(3,3) - 
               T(0,0)  * T(1,3) * T(3,1) - 
               T(1,0)  * T(0,1) * T(3,3) + 
               T(1,0)  * T(0,3) * T(3,1) + 
               T(3,0) * T(0,1) * T(1,3) - 
               T(3,0) * T(0,3) * T(1,1);

    inv(3,2) = -T(0,0)  * T(1,1) * T(3,2) + 
               T(0,0)  * T(1,2) * T(3,1) + 
               T(1,0)  * T(0,1) * T(3,2) - 
               T(1,0)  * T(0,2) * T(3,1) - 
               T(3,0) * T(0,1) * T(1,2) + 
               T(3,0) * T(0,2) * T(1,1);

    inv(0,3) = -T(0,1) * T(1,2) * T(2,3) + 
               T(0,1) * T(1,3) * T(2,2) + 
               T(1,1) * T(0,2) * T(2,3) - 
               T(1,1) * T(0,3) * T(2,2) - 
               T(2,1) * T(0,2) * T(1,3) + 
               T(2,1) * T(0,3) * T(1,2);

    inv(1,3) = T(0,0) * T(1,2) * T(2,3) - 
              T(0,0) * T(1,3) * T(2,2) - 
              T(1,0) * T(0,2) * T(2,3) + 
              T(1,0) * T(0,3) * T(2,2) + 
              T(2,0) * T(0,2) * T(1,3) - 
              T(2,0) * T(0,3) * T(1,2);

    inv(2,3) = -T(0,0) * T(1,1) * T(2,3) + 
               T(0,0) * T(1,3) * T(2,1) + 
               T(1,0) * T(0,1) * T(2,3) - 
               T(1,0) * T(0,3) * T(2,1) - 
               T(2,0) * T(0,1) * T(1,3) + 
               T(2,0) * T(0,3) * T(1,1);

    inv(3,3) = T(0,0) * T(1,1) * T(2,2) - 
               T(0,0) * T(1,2) * T(2,1) - 
               T(1,0) * T(0,1) * T(2,2) + 
               T(1,0) * T(0,2) * T(2,1) + 
               T(2,0) * T(0,1) * T(1,2) - 
               T(2,0) * T(0,2) * T(1,1);

    det = T(0,0) * inv(0,0) + T(0,1) * inv(1,0) + T(0,2) * inv(2,0) + T(0,3) * inv(3,0);

    
       
      det = 1.0 / det;

          for(int i = 0; i <= 3; i++)
             {
                for(int j = 0; j <= 3; j++)
                  {
                      invT(i,j) = inv(i,j) * det;
                  }
             }
       return invT;   
}

Eigen::Matrix4d projection (Eigen::Matrix4d B)
       {   
          Eigen::Matrix4d K;
          K = 0.5 * (B-B.transpose());
          K(3,0) = 0;
          K(3,1) = 0;
          K(3,2) = 0;
          K(3,3) = 0;
          return K;
       }

//void systeminput(const nav_msgs::Odometry::ConstPtr& msg){
void systeminput(const geometry_msgs::Twist::ConstPtr& msg){
      //nav_msgs::Odometry Odom;
      //Odom = *msg;
      //geometry_msgs::Twist speed = Odom.twist.twist;
      //lvx = speed.linear.x;
      //lvy = -speed.linear.y;
      //lvz = speed.linear.z;
      //avx = speed.angular.x;
      //avy = speed.angular.y;
      //avz = speed.angular.z;
      geometry_msgs::Twist cmd_vel;
      cmd_vel = *msg;
      lvx = cmd_vel.linear.x;
      lvy = cmd_vel.linear.y;
      lvz = cmd_vel.linear.z;
      avx = cmd_vel.angular.x;
      avy = -cmd_vel.angular.y;
      avz = -cmd_vel.angular.z;   
      
      return;
}
void poseCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg) {

      
      Eigen::Matrix3d SO_3s;
      Eigen::Matrix4d SE_3s,l,A,sump,sump1,U,se_3s,Sk,P,e;
      U << 0,avz,avy,lvx,-avz,0,avx,lvy,-avy,-avx,0,lvz,0,0,0,0;
      
      
      cout << "input =" << endl;
      cout << U << endl;
     

      SE_3s << SE_3;
     
      
      float p[4][1],s[4][1];
      float tra;
      
      Sk << inverse(SE_3);
      
      if(msg->markers.empty()) {
         ROS_INFO("No marker");
         flag = 0;
         return ;
      }
      else{
         flag = 1;
      for (int i = 0; i <= msg->markers.size()-1; i++ )
            {
               ar_track_alvar_msgs::AlvarMarker marker = msg->markers.at(i);
               geometry_msgs::Pose pose = marker.pose.pose; 
               int mark = marker.id;
               l << 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
               sump << 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
               //sump1 << 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
               if (mark > 6) 
                  {
                    n[mark][0] = 0;
                    n[mark][1] = 0;
                    n[mark][2] = 0;
                    n[mark][3] = 0;     
                  }    
              
                //cout<< "mark =" << mark ;
                cout<<endl;
                  for (int k = 0; k<=3; k++) 
                    {  
                     float sum = 0;
                     for (int j = 0; j<=3; j++ )
                         {
                              sum = sum+ Sk(k,j)*n[mark][j];
                              
                         } 
                         p[k][0] = sum; 
                         //cout << p[k][0] << " " ;
                     }
               
               s[0][0] = p[0][0]- pose.position.z;
               s[1][0] = (p[1][0] + pose.position.x) ;
               s[2][0] =(pose.position.y + p[2][0]);
               s[3][0] = 0;
              
                   for (int k = 0; k<=3; k++) 
                    {  
                     for (int j = 0; j<=3; j++ )
                       {
                          l(k,j)= s[k][0] * n[mark][j];
                          
                       }
                    }
               
                A = inverse(SE_3s).transpose()*l;
                sump = sump+0.1*A;
                //double nor;
                //nor = n[mark][0]*n[mark][0]+n[mark][1]*n[mark][1]+n[mark][2]*n[mark][2]+n[mark][3]*n[mark][3];
                //P = l/nor;
                //sump1 = sump1 + 0.02*P;  
             }
           
       
           
        se_3s =  U+ inverse(SE_3s)*projection(sump)*SE_3s; 
        //se_3s = U + inverse(SE_3s)*projection(sump)*SE_3s - projection(sump1) ; 
        for(int i = 0; i <= 2; i++)
         {
            for(int j = 0; j <= 2; j++)
               {
                  SO_3s(i,j) = SE_3s(i,j);    
               }
         }
        Eigen::Quaterniond quaternion2(SO_3s);
        x = SE_3s(0,3);
        y = SE_3s(1,3);
        z = SE_3s(2,3);
        qx = quaternion2.x();
        qy = quaternion2.y();
        qz = quaternion2.z();
        qw = quaternion2.w();
        
        pose.header.frame_id = "base_link";
              
               
               pose.pose.position.x = x;
               pose.pose.position.y = y;
               pose.pose.position.z = z;
               pose.pose.orientation.x = qx;
               pose.pose.orientation.y = qy;
               pose.pose.orientation.z = qz;
               pose.pose.orientation.w = qw;
                
        SE_3 = SE_3s*se_3s.exp();
        
        cout <<"S_now ="<<  endl;
        cout << SE_3 << endl;
        if (abs(SE_3(0,3)) > 50)
             {
                abort();
             }
         if (abs(SE_3(1,3)) > 50)
             {
                abort();
             }
         if (abs(SE_3(1,3)) > 50)
             {
                abort();
             }
        return ;
     }
    
       
}


int main(int argc, char **argv)
{
    
    SE_3 << 1,0,0,0,0,1,0,0,0,0,1,0.6,0,0,0,1;
    ros::init(argc, argv, "observer");
    ros::NodeHandle nh;
    //ros::Subscriber input = nh.subscribe("bebop2/odom",1,systeminput);
    ros::Subscriber input = nh.subscribe("bebop2/cmd_vel",1,systeminput);
    ros::Subscriber ar_pose = nh.subscribe("ar_pose_marker", 1, poseCallback);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("estimation/local_position", 1);
    ros::Publisher pub = nh.advertise<std_msgs::Float32MultiArray >("matrix_pub", 1);
    ros::Rate rate(20.0);
    while(ros::ok())
	{
             dat.data.clear();
             dat.data.push_back(flag);
             dat.data.push_back(SE_3(0,0));
             dat.data.push_back(SE_3(0,1));
             dat.data.push_back(SE_3(0,2));
             dat.data.push_back(SE_3(0,3));
             dat.data.push_back(SE_3(1,0));
             dat.data.push_back(SE_3(1,1));
             dat.data.push_back(SE_3(1,2));
             dat.data.push_back(SE_3(1,3));
             dat.data.push_back(SE_3(2,0));
             dat.data.push_back(SE_3(2,1));
             dat.data.push_back(SE_3(2,2));
             dat.data.push_back(SE_3(2,3));
             dat.data.push_back(SE_3(3,0));
             dat.data.push_back(SE_3(3,1));
             dat.data.push_back(SE_3(3,2));
             dat.data.push_back(SE_3(3,3));
             local_pos_pub.publish(pose);

             pub.publish(dat); 
             ros::spinOnce();
           
        }
    
    
        
}
