//Including ROS libraries
#include "ros/ros.h"
#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
//Including C++ nominal libraries
#include <iostream>
#include <math.h>
#include <vector>
//Including Eigen library
#include <eigen3/Eigen/Dense>

float x_est = 0;
float y_est = 0;
float z_est = -0.1;

float x1_est_dot;
float y1_est_dot;
float z1_est_dot;

float xp_est = 0;
float yp_est = 0;
float zp_est = 0;

float x2_est_dot;
float y2_est_dot;
float z2_est_dot;
    
float dist_x_est = 0;
float dist_y_est = 0;
float dist_z_est = 0;

float x3_est_dot;
float y3_est_dot;
float z3_est_dot;

float roll_est = 0;
float pitch_est = 0;
float yaw_est = 0;

float phi1_est_dot;
float theta1_est_dot;
float psi1_est_dot;

float rollRate_est = 0;
float pitchRate_est = 0;
float yawRate_est = 0;

float phi2_est_dot;
float theta2_est_dot;
float psi2_est_dot;

float dist_roll_est = 0;
float dist_pitch_est = 0;
float dist_yaw_est = 0;

float phi3_est_dot;
float theta3_est_dot;
float psi3_est_dot;

float x_error_est, y_error_est, z_error_est, roll_error_est, pitch_error_est, yaw_error_est;

float alpha1 = 0.75;
float alpha2 = 2*alpha1 - 1;
float alpha3 = 3*alpha1 - 2;

float beta1 = 1.2;
float beta2 = 2*beta1 - 1;
float beta3 = 3*beta1 - 2;

Eigen::Vector3f position;
Eigen::Vector3f attitude;
Eigen::Vector3f torques;
float thrust;

float mass = 2;
float Jxx = 0.0411;
float Jyy = 0.0478;
float Jzz = 0.0599;

Eigen::VectorXf kappa1(6);
Eigen::VectorXf kappa2(6);
Eigen::VectorXf kappa3(6);
Eigen::VectorXf epsilon1(6);
Eigen::VectorXf epsilon2(6);
Eigen::VectorXf epsilon3(6);
Eigen::VectorXf chi(6);

float step = 0.01;

void quadPosCallback(const geometry_msgs::Vector3::ConstPtr& p)
{
    position(0) = p->x;
    position(1) = p->y;
    position(2) = p->z;
}

void quadAttCallback(const geometry_msgs::Vector3::ConstPtr& a)
{
    attitude(0) = a->x;
    attitude(1) = a->y;
    attitude(2) = a->z;
}

void torquesCallback(const geometry_msgs::Vector3::ConstPtr& torq)
{
    torques(0) = torq->x;
    torques(1) = torq->y;
    torques(2) = torq->z;
}

void thrustCallback(const std_msgs::Float64::ConstPtr& t)
{
    thrust = t->data;
}

float sign(float var)
{   
    float x;
    if (var > 0)
    {
        x = 1;
    }
    else if (var<0)
    {
        x = -1;
    }
    else if (var == 0)
    {
        x = 0;
    }
    return x;
}

float sig(float a, float b) //sig = |a|^b * sign(a)
{
    float s;
    s = powf(std::abs(a), b) * sign(a);
    return s;
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "fxteso_zhang");
	ros::NodeHandle nh;
	ros::Rate loop_rate(100);	

    ros::Subscriber quad_pos_sub = nh.subscribe("quad_position", 100, &quadPosCallback);
    ros::Subscriber quad_att_sub = nh.subscribe("quad_attitude", 100, &quadAttCallback);
    ros::Subscriber torques_sub = nh.subscribe("torques", 100, &torquesCallback);
    ros::Subscriber thrust_sub = nh.subscribe("thrust", 100, &thrustCallback);
   
    ros::Publisher pos_est_pub = nh.advertise<geometry_msgs::Twist>("pos_est",100);
    ros::Publisher pos_est_only_pub = nh.advertise<geometry_msgs::Vector3>("pos_est_only",100);
    ros::Publisher dist_est_only_pub = nh.advertise<geometry_msgs::Vector3>("dist_est_only",100);
    ros::Publisher vel_est_pub = nh.advertise<geometry_msgs::Twist>("vel_est",100);
    ros::Publisher dist_est_pub = nh.advertise<geometry_msgs::Twist>("dist_est",100);

    geometry_msgs::Twist pos_est_var;
    geometry_msgs::Twist vel_est_var;
    geometry_msgs::Twist dist_est_var;
    geometry_msgs::Vector3 pos_est_only_var;
    geometry_msgs::Vector3 dist_est_only_var;

    //FXTESO gains
    kappa1 << 10, 10, 10, 10, 10, 10;
    kappa2 << 35, 35, 35, 35, 35, 35;
    kappa3 << 45, 45, 45, 45, 45, 45;
    epsilon1 << 10, 10, 10, 10, 10, 10;
    epsilon2 << 35, 35, 35, 35, 35, 35;
    epsilon3 << 45, 45, 45, 45, 45, 45;
    chi << 0.5, 0.5, 0.5, 0.5, 0.5, 0.5;

    pos_est_var.linear.x = x_est;
    pos_est_var.linear.y = y_est;
    pos_est_var.linear.z = z_est;
    pos_est_var.angular.x = roll_est;
    pos_est_var.angular.y = pitch_est;
    pos_est_var.angular.z = yaw_est;
    
    pos_est_only_var.x = x_est;
    pos_est_only_var.y = y_est;
    pos_est_only_var.z = z_est;

    pos_est_pub.publish(pos_est_var);
    pos_est_only_pub.publish(pos_est_only_var);
    ros::Duration(2.1).sleep();

    while(ros::ok())
    { 
        ///////////// roll_estimation //////////////////
        roll_error_est = attitude(0)-roll_est;
        //disturbace estimation
        phi3_est_dot  = kappa3(3)*sig(roll_error_est, alpha3) + epsilon3(3)*sig(roll_error_est, beta3) + chi(3)*sign(roll_error_est);
        dist_roll_est = dist_roll_est + phi3_est_dot*step;
        //velocity estimation
        phi2_est_dot = dist_roll_est + kappa2(3)*sig(roll_error_est, alpha2) + epsilon2(3)*sig(roll_error_est, beta2) + (torques(0)/Jxx);
        rollRate_est = rollRate_est + phi2_est_dot*step;
        //position_estimation
        phi1_est_dot = rollRate_est + kappa1(3)*sig(roll_error_est, alpha1) + epsilon1(3)*sig(roll_error_est, beta1);
        roll_est = roll_est + phi1_est_dot*step;

        ///////////// pitch_estimation //////////////////
        pitch_error_est = attitude(1)-pitch_est;
        //disturbace estimation
        theta3_est_dot  = kappa3(4)*sig(pitch_error_est, alpha3) + epsilon3(4)*sig(pitch_error_est, beta3) + chi(4)*sign(pitch_error_est);
        dist_pitch_est = dist_pitch_est + theta3_est_dot*step;
        //velocity estimation
        theta2_est_dot = dist_pitch_est + kappa2(4)*sig(pitch_error_est, alpha2) + epsilon2(4)*sig(pitch_error_est, beta2) + (torques(1)/Jyy);
        pitchRate_est = pitchRate_est + theta2_est_dot*step;
        //position_estimation
        theta1_est_dot = pitchRate_est + kappa1(4)*sig(pitch_error_est, alpha1) + epsilon1(4)*sig(pitch_error_est, beta1);
        pitch_est = pitch_est + theta1_est_dot*step;

        ///////////// yaw_estimation //////////////////
        yaw_error_est = attitude(2)-yaw_est;
        //disturbace estimation
        psi3_est_dot  = kappa3(5)*sig(yaw_error_est, alpha3) + epsilon3(5)*sig(yaw_error_est, beta3) + chi(5)*sign(yaw_error_est);
        dist_yaw_est = dist_yaw_est + psi3_est_dot*step;
        //velocity estimation
        psi2_est_dot = dist_yaw_est + kappa2(5)*sig(yaw_error_est, alpha2) + epsilon2(5)*sig(yaw_error_est, beta2) + (torques(2)/Jzz);
        yawRate_est = yawRate_est + psi2_est_dot*step;
        //position_estimation
        psi1_est_dot = yawRate_est + kappa1(5)*sig(yaw_error_est, alpha1) + epsilon1(5)*sig(yaw_error_est, beta1);
        yaw_est = yaw_est + psi1_est_dot*step;
        
        ///////////// x_estimation//////////////////
        x_error_est = position(0)-x_est;
        //disturbace estimation
        x3_est_dot  = kappa3(0)*sig(x_error_est, alpha3) + epsilon3(0)*sig(x_error_est, beta3) + chi(0)*sign(x_error_est);
        dist_x_est = dist_x_est + x3_est_dot*step;
        //velocity estimation
        x2_est_dot = dist_x_est + kappa2(0)*sig(x_error_est, alpha2) + epsilon2(0)*sig(x_error_est, beta2) + (thrust/mass)*(sin(roll_est)*sin(yaw_est) + cos(roll_est)*cos(yaw_est)*sin(pitch_est));
        xp_est = xp_est + x2_est_dot*step;
        //position_estimation
        x1_est_dot = xp_est + kappa1(0)*sig(x_error_est, alpha1) + epsilon1(0)*sig(x_error_est, beta1);
        x_est = x_est + x1_est_dot*step;

        ///////////// y_estimation//////////////////
        y_error_est = position(1)-y_est;
        //disturbace estimation
        y3_est_dot  = kappa3(1)*sig(y_error_est, alpha3) + epsilon3(1)*sig(y_error_est, beta3) + chi(1)*sign(y_error_est);
        dist_y_est = dist_y_est + y3_est_dot*step;
        //velocity estimation
        y2_est_dot = dist_y_est + kappa2(1)*sig(y_error_est, alpha2) + epsilon2(1)*sig(y_error_est, beta2) + (thrust/mass)*(cos(roll_est)*sin(yaw_est)*sin(pitch_est) - cos(yaw_est)*sin(roll_est));
        yp_est = yp_est + y2_est_dot*step;
        //position_estimation
        y1_est_dot = yp_est + kappa1(1)*sig(y_error_est, alpha1) + epsilon1(1)*sig(y_error_est, beta1);
        y_est = y_est + y1_est_dot*step;

        ///////////// z_estimation//////////////////
        z_error_est = position(2)-z_est;
        //disturbace estimation
        z3_est_dot  = kappa3(2)*sig(z_error_est, alpha3) + epsilon3(2)*sig(z_error_est, beta3) + chi(2)*sign(z_error_est);
        dist_z_est = dist_z_est + z3_est_dot*step;
        //velocity estimation
        z2_est_dot = dist_z_est + kappa2(2)*sig(z_error_est, alpha2) + epsilon2(2)*sig(z_error_est, beta2) + (thrust/mass)*(cos(roll_est)*cos(pitch_est));
        zp_est = zp_est + z2_est_dot*step;
        //position_estimation
        z1_est_dot = zp_est + kappa1(2)*sig(z_error_est, alpha1) + epsilon1(2)*sig(z_error_est, beta1);
        z_est = z_est + z1_est_dot*step;

        pos_est_var.linear.x = x_est;
        pos_est_var.linear.y = y_est;
        pos_est_var.linear.z = z_est;
        pos_est_var.angular.x = roll_est;
        pos_est_var.angular.y = pitch_est;
        pos_est_var.angular.z = yaw_est;

        vel_est_var.linear.x = xp_est;
        vel_est_var.linear.y = yp_est;
        vel_est_var.linear.z = zp_est;
        vel_est_var.angular.x = rollRate_est;
        vel_est_var.angular.y = pitchRate_est;
        vel_est_var.angular.z = yawRate_est;

        dist_est_var.linear.x = dist_x_est;
        dist_est_var.linear.y = dist_y_est;
        dist_est_var.linear.z = dist_z_est;
        dist_est_var.angular.x = dist_roll_est;
        dist_est_var.angular.y = dist_pitch_est;
        dist_est_var.angular.z = dist_yaw_est;

        pos_est_only_var.x = x_est;
        pos_est_only_var.y = y_est;
        pos_est_only_var.z = z_est;

        dist_est_only_var.x = dist_x_est;
        dist_est_only_var.y = dist_y_est;
        dist_est_only_var.z = dist_z_est;

        pos_est_pub.publish(pos_est_var);
        vel_est_pub.publish(vel_est_var);
        dist_est_pub.publish(dist_est_var);
        pos_est_only_pub.publish(pos_est_only_var);
        dist_est_only_pub.publish(dist_est_only_var);

        ros::spinOnce();
		loop_rate.sleep();
    } 

    return 0;
}
