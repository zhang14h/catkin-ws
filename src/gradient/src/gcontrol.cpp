#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/Thrust.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <sensor_msgs/Imu.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Empty.h>
#include <iostream>
#include <unsupported/Eigen/MatrixFunctions>


using namespace std;
Eigen::Matrix4d SE_3;
Eigen::Matrix4d T,Eye;
geometry_msgs::Twist cmd_vel;
int flag;



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
          K = 0.5* (B-B.transpose());
          K(3,0) = 0;
          K(3,1) = 0;
          K(3,2) = 0;
          return K;
       }
 
void matrixcallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    Eigen::Matrix4d B;
    flag = msg->data.at(0);
    cout << "flag= " << flag;
    if (!flag)
        {
           abort();
        }
    SE_3(0,0) =  msg->data.at(1);
    SE_3(0,1) =  msg->data.at(2);
    SE_3(0,2) =  msg->data.at(3);
    SE_3(0,3) =  msg->data.at(4);
    SE_3(1,0) =  msg->data.at(5);
    SE_3(1,1) =  msg->data.at(6);
    SE_3(1,2) =  msg->data.at(7);
    SE_3(1,3) =  msg->data.at(8);
    SE_3(2,0) =  msg->data.at(9);
    SE_3(2,1) =  msg->data.at(10);
    SE_3(2,2) =  msg->data.at(11);
    SE_3(2,3) =  msg->data.at(12);
    SE_3(3,0) =  msg->data.at(13);
    SE_3(3,1) =  msg->data.at(14);
    SE_3(3,2) =  msg->data.at(15);
    SE_3(3,3) =  msg->data.at(16);
    B = -0.45*inverse(T)*projection(SE_3*inverse(T))*T;
    //B = -0.5*inverse(T)*projection(SE_3*inverse(T))*T - 1.1*inverse(SE_3)*projection(SE_3*inverse(T)-Eye)*SE_3 ;

    
    cout <<endl;
    //ROS_INFO("I heard: [%f],[%f],[%f]", msg->data.at(1),msg->data.at(2),msg->data.at(6));
    if (abs(B(0,3)) >0.15)
       { 
         if (abs(B(0,3)) <1)
          {
              B(0,3) = B(0,3)/abs(B(0,3))*0.15;
          }
         else
          {
              abort();
          }
        }
     if (abs(B(1,3)) >0.15)
       { 
          if (abs(B(1,3)) <1)
            {
               B(1,3) = B(1,3)/abs(B(1,3))*0.15;
            }
          else
            {
               abort();
            }
       }
     if (abs(B(2,3)) >0.15)
       { 
          if (abs(B(2,3)) <1)
            {
               B(2,3) = B(2,3)/abs(B(2,3))*0.15;
            }
          else
            { 
               abort();
            }
       }
  
       
    cout <<"B="<<  endl;
    cout << B << endl;
    cmd_vel.linear.x = B(0,3);
    cmd_vel.linear.y = B(1,3);
    cmd_vel.linear.z = B(2,3);

    cmd_vel.angular.y = B(0,2);
    cmd_vel.angular.x =B(1,2);
    //cmd_vel.angular.y =0;
    //cmd_vel.angular.x =0;
    cmd_vel.angular.z = -B(0,1);
    return;
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "gcontrol");
    ros::NodeHandle nh;
    ros::Subscriber sub3 = nh.subscribe("matrix_pub", 100, matrixcallback);
    ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>("bebop2/cmd_vel", 1);
    ros::Publisher land_bebop = nh.advertise<std_msgs::Empty>("/bebop2/land",1);
    geometry_msgs::PoseStamped pose;
    
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);
    T << 1,0,0,0,0,1,0,0,0,0,1,0.75,0,0,0,1;
    Eye << 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1;
    while(ros::ok())
    {
        
        cmd_pub.publish(cmd_vel);
   
        ros::spinOnce();
        rate.sleep();
        
    }

    return 0;
    
}
