#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <math.h>
#include <iostream>
#include <tf/transform_datatypes.h> //转换函数头文件
#include <tf/tf.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
//#include <opencv2/core.hpp>  
//#include <cv_bridge/cv_bridge.h> 
#include "std_msgs/Empty.h"
#include "std_msgs/Float32MultiArray.h"
#include <unsupported/Eigen/MatrixFunctions>

//using namespace cv;

using namespace std;

geometry_msgs::PoseStamped pose; //建立一个该消息体类型的变量，用于存储订阅的信息
Eigen::Matrix4d A,invA,B,S,SE_3 ;
float x,y,z,qx,qy,qz,qw;
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

    //if (det == 0)
        //return false;
       
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


//回调函数，订阅的时候必须要有的，一个订阅对应一个回调函数
//作用：在接受到该消息体的内容时执行里面的内容，这里面的内容就是赋值和打印
void lp_cb(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
        flag = msg->data.at(0);
    
    //if (!flag)
        //{
           //abort();
        //}
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
        tf::Quaternion quat; //定义一个四元数
        double roll,pitch,yaw; //定义存储r\p\y的容器
        double temp;
        Eigen::Matrix3d SO_3s,R;
      
        
        
        for(int i = 0; i <= 2; i++)
         {
            for(int j = 0; j <= 2; j++)
               {
                  SO_3s(i,j) = SE_3(i,j);    
               }
         }
        Eigen::Quaterniond quaternion2(SO_3s);
        x = SE_3(0,3);
        y = SE_3(1,3);
        z = SE_3(2,3);
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
        

        Eigen::Vector3d euler_angles = SO_3s.eulerAngles(2, 1, 0);  
       
        
       
        //cout << current_position.pose.position.x <<" "<< current_position.pose.position.y <<" "<< current_position.pose.position.z << endl;
        //cout << A << endl;
        //cout << " " <<endl;
        //cout << current_position.pose.orientation.x <<" "<< current_position.pose.orientation.y <<" "<< current_position.pose.orientation.z << " "<< current_position.pose.orientation.w <<endl;
        // cout << roll << " " << pitch << " " << yaw <<endl;
        //cout << typeid(quat).name();
       return;
}

int main(int argc,char **argv)
{
        ros::init(argc,argv,"data_rcv_node"); //ros系统的初始化，最后一个参数为节点名称
        ros::NodeHandle nh;  //声明一个nh对象，NodeHandle这个类很重要，很多操作都是在这个类里面
	ros::Subscriber sub3 = nh.subscribe("matrix_pub", 100, lp_cb);

        //订阅。<>里面为模板参数，传入的是订阅的消息体类型，（）里面传入三个参数，分别是该消息体的位置、缓存大小（通常为1000）、回调函数
        ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("estimation/local_position", 10);
        
        //设置下面的循环速率 20hz
	ros::Rate rate(20.0);
        
        A<< 1,0,0,0,0,1,0,0,0,0,1,2,0,0,0,1;
        
        int16_t step_counter = 0;
        //大循环
	while(ros::ok())
	{      
              
              
              
              // pose.child_frame_id = "local2";
               
               
               local_pos_pub.publish(pose);
               
                
               ros::spinOnce(); //循环等待回调函数，用来处理所有的消息体的回调函数，类似与轮训.即所有的消息体的回调函数是在这里面触发（使用）的
                                 //ros::spinOnce();和rate.sleep();是成对存在的，两者的运行时间之和=ros::Rate rate(20.0)设定的时间;如果ros::spinOnce();没有
                                 //执行，则rate.sleep();的时间就为ros::Rate rate(20.0)设定的时间
                
                rate.sleep();//延时
       
	}
	
	return 0;
}
