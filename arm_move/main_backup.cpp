/*
 * @Author: Wei Zhifei
 * @Date: 2022-11-12 13:18:18
 */

#include <unistd.h>
#include <iostream>
#include <string>
#include <vector>
#include <memory>


#include "Me_arm.hpp"
#include "model_equa_solver.hpp"

#include <ros/ros.h>
#include <arm_move/arm_motor_vel.h>
// #include <arm_move/arm_scrapAction.h>
#include <robot_bringup/joint.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Float64.h>
// #include <actionlib/server/simple_action_server.h>

// typedef actionlib::SimpleActionServer<arm_move::arm_scrapAction> Server;

// robot_bringup::joint _joint_deg;    //encoder degree not alpha and beta
// robot_bringup::joint _target_deg;   //encoder degree not alpha and beta

// /* get the tow joint's degree */
// void joint_callback(const robot_bringup::joint::ConstPtr &joint_msg)
// {
//     _joint_deg = *joint_msg;
// }	


/* get target and calculate the degree of two joints*/
void target_point_callback(const geometry_msgs::Point::ConstPtr &target_msg, ros::Publisher &big_pub, ros::Publisher &small_pub)
{
    robot_bringup::joint target_radian;    //alpha and beta not encoder degree
    std_msgs::Float64 alpha_deg, beta_deg;
    std_msgs::Float64 big_deg, small_deg;
    bool res = get_started(target_msg->x - 0.035, target_msg->z, target_radian);
    alpha_deg.data = (target_radian.bigJoint_deg)/3.1415926*180;
    beta_deg.data = (target_radian.smallJoint_deg)/3.1415926*180;

    big_deg.data = 90.0 - alpha_deg.data;
    small_deg.data = alpha_deg.data - beta_deg.data - 90.0; 

    big_pub.publish(big_deg);   // alpha and beta degree
    small_pub.publish(small_deg);  

    ROS_INFO("big: %f deg    small: %f deg", big_deg.data, small_deg.data);
}	


/* get vel and transform it from topic to motor driver */
void arm_callback(const arm_move::arm_motor_vel::ConstPtr &motor_vel, Me_arm &myarm)
{
    myarm.elmo_setVelocity(1,motor_vel->almo_one);
    usleep(1000);
    myarm.elmo_setVelocity(2,motor_vel->almo_two);
    if(motor_vel->stepper_one != 0)
	    myarm.Stepper_run_withVelocity(0xC1,motor_vel->stepper_one);
    else
	    myarm.Stepper_stop(0xC1);
    if(motor_vel->stepper_two != 0)
	    myarm.Stepper_run_withVelocity(0xC2,motor_vel->stepper_two);
    else
	    myarm.Stepper_stop(0xC1);
}	

void big_effort_callback(const std_msgs::Float64::ConstPtr &big_vel, Me_arm &myarm)
{
    myarm.elmo_setVelocity(2,(int32_t)((big_vel->data)*100));
}

void small_effort_callback(const std_msgs::Float64::ConstPtr &small_vel, Me_arm &myarm)
{
    myarm.elmo_setVelocity(1,(int32_t)((small_vel->data)*100));
}

void big_joint_callback(const std_msgs::Float64::ConstPtr &big_joint_deg_msg, double &_big_joint_deg)
{
	_big_joint_deg = big_joint_deg_msg->data;
}

void small_joint_callback(const std_msgs::Float64::ConstPtr &small_joint_deg_msg, double &_small_joint_deg)
{
	_small_joint_deg = small_joint_deg_msg->data;
}


/* * * * * * * * * * * *
 * 机械臂抓取动作流程
 * * * * * * * * * * * */
 /*
  *action 接口
  */
// void execute(const arm_move::arm_scrapGoalConstPtr& arm_goal, Server* as)
// {
//     ros::Rate r(1); /* 设置运行频率，这里设置为1hz */
//     arm_move::arm_scrapFeedback feedback;    /* 创建一个feedback对象 */
    
//     for (int count = 0; count < (arm_goal->x); ++count)
//     {
//         feedback.complete_percent = count;
//         as->publishFeedback(feedback);

//         r.sleep();
//     }

//     as->setSucceeded();   /* 发送result */
// }

/* main */
int main(int argc,char *argv[])
{
    Me_arm myarm;
	double big_joint_deg;//大臂编码器角度。
	double small_joint_deg;//小臂编码器角度
	double alpha_deg,beta_deg;//模型中的两个角

    myarm.elmo_setmod(1,Velocity_mode);
    myarm.elmo_enable(1);

    myarm.elmo_setmod(2,Velocity_mode);
    myarm.elmo_enable(2);

    myarm.Stepper_stop(0XC1);

    ros::init(argc, argv, "arm_move_base");
	ros::NodeHandle nh /*("~")*/;

	ros::Publisher 	big_target_joint_deg_pub = nh.advertise<std_msgs::Float64>("/big_target_joint_deg",1);
	ros::Publisher 	small_target_joint_deg_pub = nh.advertise<std_msgs::Float64>("/small_target_joint_deg",1);
	ros::Publisher 	current_point_pub = nh.advertise<geometry_msgs::Point>("/current_point",1);

    ros::Subscriber vel_sub = nh.subscribe<arm_move::arm_motor_vel>("/arm_motor_vel", 1, boost::bind(&arm_callback, _1, myarm));
    ros::Subscriber big_effort_sub = nh.subscribe<std_msgs::Float64>("/big_control_effort", 1, boost::bind(&big_effort_callback, _1, myarm));
    ros::Subscriber small_effort_sub = nh.subscribe<std_msgs::Float64>("/small_control_effort", 1, boost::bind(&small_effort_callback, _1, myarm));
    // ros::Subscriber joint_sub = nh.subscribe<robot_bringup::joint>("/joint_deg", 1, joint_callback);
    ros::Subscriber big_joint_sub = nh.subscribe<std_msgs::Float64>("/big_joint_deg", 10, boost::bind(&big_joint_callback, _1, big_joint_deg));
	ros::Subscriber small_joint_sub = nh.subscribe<std_msgs::Float64>("/small_joint_deg", 10, boost::bind(&small_joint_callback, _1, small_joint_deg));
    ros::Subscriber point_sub = nh.subscribe<geometry_msgs::Point>("/target_point", 10, boost::bind(&target_point_callback, _1, big_target_joint_deg_pub, small_target_joint_deg_pub));	
    /* 创建action server对象; */
    // Server server(nh, "arm_scrap", boost::bind(&execute, _1, &server), false);
    // server.start();

    ros::AsyncSpinner s(4);
  	s.start(); 
	while(ros::ok())
	{
		/*这里解算出目标点的坐标，做观测用*/
		
		sleep(1);
	}	      
}
