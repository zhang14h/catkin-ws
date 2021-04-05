using namespace std;
#include <iostream>
#include <stdio.h>
#include <ros/ros.h>
//topic
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>

std_msgs::Empty take_off,land;							//Variable to take_off and land
geometry_msgs::Twist cmd_vel;	


float x,y,z;

int main(int argc, char **argv)
{   
    //ros::Rate rate(20.0);
    ros::init(argc, argv, "bebop_run");
    ros::NodeHandle nh;	
    
    ros::Publisher takeoff_bebop = nh.advertise<std_msgs::Empty>("/bebop2/takeoff",1);		//Publish data to the take-off topic
    ros::Publisher land_bebop = nh.advertise<std_msgs::Empty>("/bebop2/land",1);
    ros::Publisher cmd_pub    = nh.advertise<geometry_msgs::Twist>("bebop2/cmd_vel", 1);
    

    ros::Rate rate(20.0);

    ros::Duration(3).sleep();
    land_bebop.publish(std_msgs::Empty());  
    
    rate.sleep();


    ros::spinOnce();
    


    return 0;
}
