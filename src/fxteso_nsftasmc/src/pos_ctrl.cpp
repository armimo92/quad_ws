//Including ROS libraries
#include "ros/ros.h"
#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
//Including C++ nominal libraries
#include <iostream>
#include <math.h>
#include <vector>
//Including Eigen library
#include <eigen3/Eigen/Dense>

Eigen::Vector3f position_est;
Eigen::Vector3f velocity_est;
Eigen::Vector3f attitude_est;
Eigen::Vector3f disturbance_est;
Eigen::Vector3f posRef;
Eigen::Vector3f velRef;
Eigen::Vector3f attRef;
Eigen::Vector3f error_pos;
Eigen::Vector3f error_vel;

Eigen::Vector3f sigma;
Eigen::Vector3f xi_1;
Eigen::Vector3f xi_2;
Eigen::Vector3f lambda;
Eigen::Vector3f gam;

Eigen::Vector3f asmc;
Eigen::Vector3f K1;
Eigen::Vector3f K1_dot;
Eigen::Vector3f K2;
Eigen::Vector3f k_reg;
Eigen::Vector3f kmin;
Eigen::Vector3f mu;

float roll_des=0;
float pitch_des=0;
float yaw_des;
float step_size = 0.01;

float m = 2;
float thrust;
float g = 9.81;
float z_ddot_des;
float roll_des_arg;
float pitch_des_arg;

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

/*
void posCallback(const geometry_msgs::Vector3::ConstPtr& p)
{
    position(0) = p->x;
    position(1) = p->y;
    position(2) = p->z;
}

void velIFCallback(const geometry_msgs::Vector3::ConstPtr& v)
{
    velocity_IF(0) = v->x;
    velocity_IF(1) = v->y;
    velocity_IF(2) = v->z;
}

void attCallback(const geometry_msgs::Vector3::ConstPtr& a)
{
    attitude(0) = a->x;
    attitude(1) = a->y;
    attitude(2) = a->z;
}
*/

void posEstCallback(const geometry_msgs::Twist::ConstPtr& pE)
{
    position_est(0) = pE->linear.x;
    position_est(1) = pE->linear.y;
    position_est(2) = pE->linear.z;
    attitude_est(0) = pE->angular.x;
    attitude_est(1) = pE->angular.y;
    attitude_est(2) = pE->angular.z;
}

void velEstCallback(const geometry_msgs::Twist::ConstPtr& vE)
{
    velocity_est(0) = vE->linear.x;
    velocity_est(1) = vE->linear.y;
    velocity_est(2) = vE->linear.z;
    
}

void distEstCallback(const geometry_msgs::Twist::ConstPtr& dE)
{
    disturbance_est(0) = dE->linear.x;
    disturbance_est(1) = dE->linear.y;
    disturbance_est(2) = dE->linear.z;
    
}

void posRefCallback(const geometry_msgs::Vector3::ConstPtr& pr)
{
    posRef(0) = pr->x;
    posRef(1) = pr->y;
    posRef(2) = pr->z;
}

void yawRefCallback(const std_msgs::Float64::ConstPtr& yD)
{
    yaw_des = yD->data;
}

void velRefCallback(const geometry_msgs::Quaternion::ConstPtr& vr)
{
    velRef(0) = vr->x;
    velRef(1) = vr->y;
    velRef(2) = vr->z;
    z_ddot_des = vr->w;
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "pos_ctrl");
	ros::NodeHandle nh;
	ros::Rate loop_rate(100);	

    /*
    ros::Subscriber pos_sub = nh.subscribe("quad_position", 100, &posCallback);
    ros::Subscriber vel_IF_sub = nh.subscribe("quad_velocity_IF", 100, &velIFCallback);
    ros::Subscriber att_sub = nh.subscribe("quad_attitude", 100, &attCallback);
    */

    ros::Subscriber pos_est_sub = nh.subscribe("pos_est", 100, &posEstCallback);
    ros::Subscriber vel_est_sub = nh.subscribe("vel_est", 100, &velEstCallback);
    ros::Subscriber dist_est_sub = nh.subscribe("dist_est", 100, &distEstCallback);

    ros::Subscriber posRef_sub = nh.subscribe("desired_position", 100, &posRefCallback);
    ros::Subscriber velRef_sub = nh.subscribe("desired_velocity", 100, &velRefCallback);
    ros::Subscriber yawDes_sub = nh.subscribe("desired_yaw", 100, &yawRefCallback);

    ros::Publisher att_des_pub = nh.advertise<geometry_msgs::Vector3>("desired_attitude",100);    
    ros::Publisher thrust_pub = nh.advertise<std_msgs::Float64>("thrust",100);
    ros::Publisher K1_pos_pub = nh.advertise<geometry_msgs::Vector3>("K1_pos",100);
    ros::Publisher error_pos_pub = nh.advertise<geometry_msgs::Vector3>("error_pos",100);
    ros::Publisher error_vel_pub = nh.advertise<geometry_msgs::Vector3>("error_vel",100);
    ros::Publisher sigma_pos_pub = nh.advertise<geometry_msgs::Vector3>("sigma_pos",100);

    ros::Publisher pitch_des_arg_pub = nh.advertise<std_msgs::Float64>("pitch_des_arg",100);
    ros::Publisher roll_des_arg_pub = nh.advertise<std_msgs::Float64>("roll_des_arg",100);

    geometry_msgs::Vector3 att_des_var;
    geometry_msgs::Vector3 K1_pos_var;
    geometry_msgs::Vector3 error_pos_var;
    geometry_msgs::Vector3 error_vel_var;
    geometry_msgs::Vector3 sigma_pos_var;
    std_msgs::Float64 thrust_var;
    std_msgs::Float64 pitch_des_arg_var;
    std_msgs::Float64 roll_des_arg_var;

    
    xi_1 << 0.2,0.2,2;
    xi_2 << 2,2,2;
    gam << 4/3,4/3,4/3; // 1 < gamma < 2
    lambda << 1.5,1.5,1.5; // lambda > gamma    
    K1 << 0,0,0;
    K1_dot << 0,0,0;
    K2 << 0.6,0.6,1;
    k_reg << 0.15,0.15,0.8;
    mu << 0.03,0.03,0.03;
    kmin << 0.01,0.01,0.01; 

    thrust_var.data = 0;
    thrust_pub.publish(thrust_var);   

    att_des_var.x = 0;
    att_des_var.y = 0;
    att_des_var.z = 0;
    att_des_pub.publish(att_des_var);
    ros::Duration(2).sleep();

    while(ros::ok())
    { 

        //Nonsingular terminal sliding surface
        for (int i = 0; i <= 2; i++)
        {
            //Error 
            error_pos(i) = position_est(i) - posRef(i);
            error_vel(i)= velocity_est(i) - velRef(i);

            sigma(i) = error_pos(i) + xi_1(i) * powf(std::abs(error_pos(i)),lambda(i)) * sign(error_pos(i)) + xi_2(i) * powf(std::abs(error_vel(i)),(gam(i))) * sign(error_vel(i));
            
            if (K1(i)>kmin(i))
            {
                K1_dot(i) = k_reg(i)*sign(std::abs(sigma(i))-mu(i));
            }
            else
            {
                K1_dot(i) = kmin(i);
            }

            K1(i) = K1(i) + step_size*K1_dot(i);
            asmc(i) = -K1(i) * powf(std::abs(sigma(i)),0.5) * sign(sigma(i)) - K2(i) * sigma(i);
        }

        thrust = (m/(cos(attitude_est(0))*cos(attitude_est(1)))) * (asmc(2) - disturbance_est(2) + z_ddot_des - (1/(xi_2(2)*gam(2)))*powf(std::abs(error_vel(2)), 2-gam(2))*sign(error_vel(2)) - (1/(xi_2(2)*gam(2)))*powf(std::abs(error_vel(2)), 2-gam(2))*sign(error_vel(2))*xi_1(2)*lambda(2)*powf(std::abs(error_pos(2)), lambda(2)-1));

        if (thrust > 0)
        {
            thrust = 0;
        }
        else if (thrust < -30)
        {
            thrust = -30;
        }
        
        roll_des_arg = (m/thrust)*(sin(yaw_des)*asmc(0) - cos(yaw_des)*asmc(1));
        if (roll_des_arg>1)
        {
            roll_des_arg = 1;
        }
        else if (roll_des_arg<-1)
        {
            roll_des_arg = -1;
        }

        roll_des = asin(roll_des_arg);

        pitch_des_arg = ((m/thrust)*asmc(0) - sin(yaw_des)*sin(roll_des)) / (cos(yaw_des)*cos(roll_des));
        if (pitch_des_arg>1)
        {
            pitch_des_arg = 1;
        }
        else if (pitch_des_arg<-1)
        {
            pitch_des_arg = -1;
        }

        pitch_des = asin(pitch_des_arg);

        att_des_var.x = roll_des;
        att_des_var.y = pitch_des;
        att_des_var.z = yaw_des;

        thrust_var.data = thrust;

        error_pos_var.x = error_pos(0);
        error_pos_var.y = error_pos(1);
        error_pos_var.z = error_pos(2);

        error_vel_var.x = error_vel(0);
        error_vel_var.y = error_vel(1);
        error_vel_var.z = error_vel(2);

        K1_pos_var.x = K1(0);
        K1_pos_var.y = K1(1);
        K1_pos_var.z = K1(2);

        sigma_pos_var.x = sigma(0);
        sigma_pos_var.y = sigma(1);
        sigma_pos_var.z = sigma(2);

        pitch_des_arg_var.data = pitch_des_arg;
        roll_des_arg_var.data= roll_des_arg;


        att_des_pub.publish(att_des_var);
        thrust_pub.publish(thrust_var);
        K1_pos_pub.publish(K1_pos_var);
        error_pos_pub.publish(error_pos_var);
        error_vel_pub.publish(error_vel_var);
        sigma_pos_pub.publish(sigma_pos_var);
        pitch_des_arg_pub.publish(pitch_des_arg_var);
        roll_des_arg_pub.publish(roll_des_arg_var);


        ros::spinOnce();
		loop_rate.sleep();
    }

    return 0;
}
