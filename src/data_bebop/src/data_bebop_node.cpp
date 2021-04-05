#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include <geometry_msgs/TwistStamped.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Empty.h>
#include <iostream>
#include <fstream>


using namespace std;
Eigen::Matrix4d SE_3;
Eigen::Matrix4d T;
int flag,counter;
double xdiff,ydiff,zdiff,rdiff;
double dx,dy,dz,dr;
ofstream p;
std_msgs::Float32MultiArray datx,daty,datz,datr;

 
void matrixcallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
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

    //xdiff = abs(T(0,3)-SE_3(0,3));
    //ydiff = abs(T(1,3)-SE_3(1,3));
    //zdiff = abs(T(2,3)-SE_3(2,3));
    xdiff = (SE_3(0,3)-T(0,3));
    ydiff = (SE_3(1,3)-T(1,3));
    zdiff = (SE_3(2,3)-T(2,3));
    //rdiff = pow((T(0,0)-SE_3(0,0)),2)+pow((T(0,1)-SE_3(0,1)),2)+pow((T(0,2)-SE_3(0,2)),2)+pow((T(1,0)-SE_3(1,0)),2)+pow((T(1,1)-SE_3(1,1)),2)
 //+pow((T(1,2)-SE_3(1,2)),2)+pow((T(2,0)-SE_3(2,0)),2)+pow((T(2,1)-SE_3(2,1)),2)+pow((T(2,2)-SE_3(2,2)),2);
    //rdiff = sqrt(rdiff);    
    rdiff = SE_3(0,1)-T(0,1);
   
    
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "data_bebop_node");
    ros::NodeHandle nh;
    ros::Subscriber sub3 = nh.subscribe("matrix_pub", 100, matrixcallback);
    //ros::Publisher pub = nh.advertise<std_msgs::Float32MultiArray >("data", 1);
    
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);
    T << 1,0,0,0,0,1,0,0,0,0,1,0.75,0,0,0,1;  
    
    counter = 0;

   while(ros::ok())
    {
         datx.data.push_back(xdiff);
         daty.data.push_back(ydiff);
         datz.data.push_back(zdiff);
         datr.data.push_back(rdiff);
         counter ++;
         cout << counter <<endl;
         if (counter == 6700)
            {
             //pub.publish(datx); 
             counter = 0;
             
             p.open("data.csv", ios::out); 
             for (int i=0; i<6700; i++)
                {
                  dx =  datx.data.at(i);
                  dy =  daty.data.at(i);
                  dz =  datz.data.at(i);
                  dr =  datr.data.at(i);
                  p << dx << ",";
                  p << dy << ",";
                  p << dz << ",";
                  p << dr << endl;
                }
             datx.data.clear();   
             daty.data.clear();
             datz.data.clear();
             datr.data.clear();               
             p.close();     
            }
   
        ros::spinOnce();
        rate.sleep();
       
    
        
        
    }
}
