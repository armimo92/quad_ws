//Including ROS libraries
#include "ros/ros.h"
#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
//Including C++ nominal libraries
#include <iostream>
#include <math.h>
#include <vector>
//Including Eigen library
#include <eigen3/Eigen/Dense>

float x_des = 0;
float y_des = 0;
float z_des = 0;
float yaw_des = 0;
float x_dot_des,y_dot_des,z_dot_des,z_ddot_des, yawRate;
float step = 0.01;
int i = 0;
float t;

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "poseRef");
	ros::NodeHandle nh;
	ros::Rate loop_rate(100);	
   
    ros::Publisher posRef_pub = nh.advertise<geometry_msgs::Vector3>("desired_position",100);
    ros::Publisher velRef_pub = nh.advertise<geometry_msgs::Quaternion>("desired_velocity",100);
    ros::Publisher yawRef_pub = nh.advertise<std_msgs::Float64>("desired_yaw",100);
    ros::Publisher att_vel_des_pub = nh.advertise<geometry_msgs::Vector3>("desired_attitude_velocity",100);

    geometry_msgs::Vector3 posRef_var;
    geometry_msgs::Quaternion velRef_var;
    std_msgs::Float64 yawRef_var;
    geometry_msgs::Vector3 attVelRef_var;

    posRef_var.x = 0;
    posRef_var.y = 0;
    posRef_var.z = 0;

    velRef_var.x = 0;
    velRef_var.y = 0;
    velRef_var.z = 0;
    velRef_var.w = 0;

    yawRef_var.data = 0;

    attVelRef_var.x = 0;
    attVelRef_var.y = 0;
    attVelRef_var.z = 0;

    posRef_pub.publish(posRef_var);
    velRef_pub.publish(velRef_var);
    yawRef_pub.publish(yawRef_var);
    att_vel_des_pub.publish(attVelRef_var);
    ros::Duration(2).sleep();


    while(ros::ok())
    { 
        t = i*step;

        /*
        x_des = 1;
        y_des = 0;
        z_des = -2;

        x_dot_des = 0;
        y_dot_des = 0;
        z_dot_des = 0;

        yaw_des = 0;

        */

        x_dot_des = 0.6 * cos(0.3*t);
        y_dot_des = 0.6 * sin(0.3*t);
        z_dot_des = 0;
		yawRate = 0;

	    x_des = x_des + x_dot_des*step;
        y_des = y_des + y_dot_des*step;
        z_des = -2;
		yaw_des = yaw_des + yawRate*step;		
        
        z_ddot_des = 0;


        posRef_var.x = x_des;
        posRef_var.y = y_des;
        posRef_var.z = z_des;

        velRef_var.x = x_dot_des;
        velRef_var.y = y_dot_des;
        velRef_var.z = z_dot_des;
        velRef_var.w = z_ddot_des;

        yawRef_var.data = yaw_des;

        attVelRef_var.x = 0;
        attVelRef_var.y = 0;
        attVelRef_var.z = 0;

        posRef_pub.publish(posRef_var);
        velRef_pub.publish(velRef_var);
        yawRef_pub.publish(yawRef_var);
        att_vel_des_pub.publish(attVelRef_var);

        i = i+1;

        ros::spinOnce();
		loop_rate.sleep();
    }

    return 0;
}
