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

Eigen::Vector3f attVelRef;
Eigen::Vector3f attRef;
Eigen::Vector3f error_att;
Eigen::Vector3f error_attVel;
Eigen::Vector3f attitude_est;
Eigen::Vector3f attitude_vel_est;
Eigen::Vector3f disturbance_est;

Eigen::Vector3f sigma;
Eigen::Vector3f xi_1;
Eigen::Vector3f xi_2;
Eigen::Vector3f gam;
Eigen::Vector3f lambda;

Eigen::Vector3f asmc;
Eigen::Vector3f K1;
Eigen::Vector3f K1_dot;
Eigen::Vector3f K2;
Eigen::Vector3f k_reg;
Eigen::Vector3f kmin;
Eigen::Vector3f mu;

Eigen::Vector3f tau;

float step_size = 0.01;

float Jxx = 0.0411;
float Jyy = 0.0478;
float Jzz = 0.0599;


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

void attDesCallback(const geometry_msgs::Vector3::ConstPtr& aD)
{
    attRef(0) = aD->x;
    attRef(1) = aD->y;
    attRef(2) = aD->z;
}

void attVelDesCallback(const geometry_msgs::Vector3::ConstPtr& avD)
{
    attVelRef(0) = avD->x;
    attVelRef(1) = avD->y;
    attVelRef(2) = avD->z;
}

void posEstCallback(const geometry_msgs::Twist::ConstPtr& pE)
{
    attitude_est(0) = pE->angular.x;
    attitude_est(1) = pE->angular.y;
    attitude_est(2) = pE->angular.z;
}

void velEstCallback(const geometry_msgs::Twist::ConstPtr& vE)
{
    attitude_vel_est(0) = vE->angular.x;
    attitude_vel_est(1) = vE->angular.y;
    attitude_vel_est(2) = vE->angular.z;
}

void distEstCallback(const geometry_msgs::Twist::ConstPtr& dE)
{
    disturbance_est(0) = dE->angular.x;
    disturbance_est(1) = dE->angular.y;
    disturbance_est(2) = dE->angular.z;
    
}

/*
void attCallback(const geometry_msgs::Vector3::ConstPtr& a)
{
    attitude(0) = a->x;
    attitude(1) = a->y;
    attitude(2) = a->z;
}

void attVelCallback(const geometry_msgs::Vector3::ConstPtr& av)
{
    attitude_vel(0) = av->x;
    attitude_vel(1) = av->y;
    attitude_vel(2) = av->z;
}
*/

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "att_ctrl");
	ros::NodeHandle nh;
	ros::Rate loop_rate(100);	

    ros::Subscriber att_des_sub = nh.subscribe("desired_attitude", 100, &attDesCallback);
    ros::Subscriber attVel_des_sub = nh.subscribe("desired_attitude_velocity", 100, &attVelDesCallback);

    ros::Subscriber pos_est_sub = nh.subscribe("pos_est", 100, &posEstCallback);
    ros::Subscriber vel_est_sub = nh.subscribe("vel_est", 100, &velEstCallback);
    ros::Subscriber dist_est_sub = nh.subscribe("dist_est", 100, &distEstCallback);


    /*
    ros::Subscriber att_sub = nh.subscribe("quad_attitude", 100, &attCallback);
    ros::Subscriber attVel_sub = nh.subscribe("quad_attitude_velocity", 100, &attVelCallback);
    */

    ros::Publisher torque_pub = nh.advertise<geometry_msgs::Vector3>("torques",100);
    ros::Publisher sigma_att_pub = nh.advertise<geometry_msgs::Vector3>("sigma_att",100);
    ros::Publisher error_att_pub = nh.advertise<geometry_msgs::Vector3>("error_att",100);

    geometry_msgs::Vector3 torque_var;
    geometry_msgs::Vector3 sigma_att_var;
    geometry_msgs::Vector3 error_att_var;

    xi_1 << 0.2, 0.2, 0.5; //xi_1 > 0;
    xi_2 << 0.5, 0.5, 1.2; //xi_2 > 0;
    gam << 4/3,4/3,4/3; // 1 < gamma < 2
    lambda << 1.5,1.5,1.5; // lambda > gamma    
    K1 << 0,0,0;
    K1_dot << 0,0,0;
    K2 << 0.3, 0.3, 0.01;
    k_reg << 1, 1, 1;
    mu << 0.1, 0.1, 0.2;
    kmin << 3, 3, 1;  

    while(ros::ok())
    { 
        //Error 
        error_att = attitude_est - attRef;
        error_attVel= attitude_vel_est - attVelRef;

        //Nonsingular terminal sliding surface
        for (int i = 0; i <= 2; i++)
        {
            sigma(i) = error_att(i) + xi_1(i) * powf(std::abs(error_att(i)),lambda(i)) * sign(error_att(i)) + xi_2(i) * powf(std::abs(error_attVel(i)),(gam(i))) * sign(error_attVel(i));

            if (K1(i)>kmin(i))
            {
                K1_dot(i) = k_reg(i)*sign(std::abs(sigma(i))-mu(i));
            }
            else
            {
                K1_dot(i) = kmin(i);
            }

            K1(i) = K1(i) + step_size*K1_dot(i);
            asmc(i) = -K1(i) * powf(std::abs(sigma(i)),0.5) * sign(sigma(i)) - K2(i) * sigma(i) - disturbance_est(i);
        }

        tau(0) = Jxx * ( - (1/(xi_2(0)*gam(0)))*powf(std::abs(error_attVel(0)), 2-gam(0))*sign(error_attVel(0)) - (1/(xi_2(0)*gam(0)))*powf(std::abs(error_attVel(0)), 2-gam(0))*sign(error_attVel(0))*xi_1(0)*lambda(0)*powf(std::abs(error_att(0)), lambda(0)-1) + asmc(0) );
        tau(1) = Jyy * ( - (1/(xi_2(1)*gam(1)))*powf(std::abs(error_attVel(1)), 2-gam(1))*sign(error_attVel(1)) - (1/(xi_2(1)*gam(1)))*powf(std::abs(error_attVel(1)), 2-gam(1))*sign(error_attVel(1))*xi_1(1)*lambda(1)*powf(std::abs(error_att(1)), lambda(1)-1) + asmc(1) );
        tau(2) = Jzz * ( - (1/(xi_2(2)*gam(2)))*powf(std::abs(error_attVel(2)), 2-gam(2))*sign(error_attVel(2)) - (1/(xi_2(2)*gam(2)))*powf(std::abs(error_attVel(2)), 2-gam(2))*sign(error_attVel(2))*xi_1(0)*lambda(2)*powf(std::abs(error_att(2)), lambda(2)-1) + asmc(2) );

        torque_var.x = tau(0);
        torque_var.y = tau(1);
        torque_var.z = tau(2);

        sigma_att_var.x = sigma(0);
        sigma_att_var.y = sigma(1);
        sigma_att_var.z = sigma(2);

        error_att_var.x = error_att(0);
        error_att_var.y = error_att(1);
        error_att_var.z = error_att(2);
        
        torque_pub.publish(torque_var);
        sigma_att_pub.publish(sigma_att_var);
        error_att_pub.publish(error_att_var);
       
        ros::spinOnce();
		loop_rate.sleep();
    }

    return 0;
}
