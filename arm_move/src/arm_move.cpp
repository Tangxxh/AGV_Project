/*
 * @Author: Wei Zhifei
 * @Date: 2022-11-12 13:18:18
 * 未使用C++类的形式写，有C风格。全局变量较少。
 */

#include <unistd.h>
#include <iostream>
#include <cmath>

#include <ros/ros.h>

#include "Me_arm.hpp"
#include "model_equa_solver.hpp"

#include <arm_move/arm_motor_vel.h>
#include <robot_bringup/joint.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

 // robot_bringup::joint _joint_deg;    //encoder degree not alpha and beta
 // robot_bringup::joint _target_deg;   //encoder degree not alpha and beta

 // /* get the tow joint's degree */
 // void joint_callback(const robot_bringup::joint::ConstPtr &joint_msg)
 // {
 //     _joint_deg = *joint_msg;
 // }	


double big_effort, small_effort, LR_effort;//PID输出值
double big_joint_deg;//大臂编码器角度。
double small_joint_deg;//小臂编码器角度。
std_msgs::Bool arm_is_ok;//
std_msgs::Bool pid_enable;
double targetXPoint, targetZPoint;

// std_msgs::Bool init_flag;

bool scraped_flag = false; //记录夹爪状态的变量 false是松开状态 true是夹紧状态，需要从上位机读取参数
bool arm_can_be_control;
bool targetPointArrivedFlag = false;
ros::Publisher 	pid_enable_pub;
ros::Publisher  isok_pub;
ros::Publisher  log_pub;
ros::Subscriber emergency_sub;

void log_to_pc(const char* log)
{
    std_msgs::String logPC;
    logPC.data = log;
    log_pub.publish(logPC);
}


 /* get target and calculate the degree of two joints*/
void target_point_callback(const geometry_msgs::Point::ConstPtr& target_msg, ros::Publisher& big_pub, ros::Publisher& small_pub)
{
    arm_is_ok.data = false;
    isok_pub.publish(arm_is_ok);
    targetPointArrivedFlag = false;

    robot_bringup::joint target_radian;    //alpha and beta not encoder degree
    std_msgs::Float64 alpha_deg, beta_deg;
    std_msgs::Float64 big_deg, small_deg;

    targetXPoint = target_msg->x;
    targetZPoint = target_msg->z;

    bool res = get_started(target_msg->x - 0.035, target_msg->z, target_radian);
    alpha_deg.data = (target_radian.bigJoint_deg) / 3.1415926 * 180;
    beta_deg.data = (target_radian.smallJoint_deg) / 3.1415926 * 180;

    big_deg.data = 90.0 - alpha_deg.data;
    small_deg.data = alpha_deg.data - beta_deg.data - 90.0;

    if(((big_deg.data + 90.0) + 20.0 < (90.0f - small_deg.data)) || (small_deg.data < -48.0) || (small_deg.data > 16.0) || (big_deg.data > 63.0) || (big_deg.data < -8.0f))
    {
        ROS_ERROR("big: %f deg  small: %f deg", big_deg.data, small_deg.data);
        log_to_pc("[arm_move] error : joint_deg is exceed threshold!");
    }
    else
    {
        big_pub.publish(big_deg);   // alpha and beta degree
        small_pub.publish(small_deg);

        pid_enable.data = true;
        pid_enable_pub.publish(pid_enable);

        ROS_INFO("big: %f deg    small: %f deg", big_deg.data, small_deg.data);
    }
}


/* get vel and transform it from topic to motor driver */
void arm_callback(const arm_move::arm_motor_vel::ConstPtr& motor_vel, Me_arm& myarm)
{
    myarm.elmo_setVelocity(1, motor_vel->almo_one);
    usleep(1000);
    myarm.elmo_setVelocity(2, motor_vel->almo_two);
    if (motor_vel->stepper_one > 0)
        myarm.Stepper_run_withVelocity(BIG_LR, 1, (float_t)motor_vel->stepper_one);
    else if (motor_vel->stepper_one < 0)
        myarm.Stepper_run_withVelocity(BIG_LR, -1, (float_t)(-(motor_vel->stepper_one)));
    else
        myarm.Stepper_stop(BIG_LR);
    if (motor_vel->stepper_two > 0)
        myarm.Gripper_clamp();
    else if (motor_vel->stepper_two < 0)
        myarm.Gripper_loosen();
    else
        myarm.Stepper_stop(SCRAP);
}

void return_zero_callback(const std_msgs::BoolConstPtr &return_msg, Me_arm &myarm)
{
    if(return_msg->data)
    {
        myarm.Stepper_findzeroPosition(BIG_LR);//回零
    }
}

void LR_abs_positon_callback(const std_msgs::Int32ConstPtr &pos_msg, Me_arm &myarm)
{
    myarm.Stepper_run_withabsPosition(BIG_LR, pos_msg->data, 100);
}

void LR_rela_positon_callback(const std_msgs::Int32ConstPtr& rela_msg, Me_arm& myarm)
{
    if (rela_msg->data > 0)
        myarm.Stepper_run_withrelPosition(BIG_LR,rela_msg->data,1,100);
    else if (rela_msg->data < 0)
        myarm.Stepper_run_withrelPosition(BIG_LR, -rela_msg->data, -1,100);
    else
        myarm.Stepper_stop(BIG_LR);
}

void big_effort_callback(const std_msgs::Float64::ConstPtr& big_vel, Me_arm& myarm)
{
    if(arm_can_be_control && !targetPointArrivedFlag)
    {
        big_effort = big_vel->data;
        myarm.elmo_setVelocity(2, (int32_t)((big_vel->data) * 100));
    }
    else
    {
        myarm.elmo_setVelocity(2, 0);
        myarm.elmo_setVelocity(1, 0);
        ROS_ERROR("------------------BIG stop");
        arm_is_ok.data = true;
        pid_enable.data = false;
        isok_pub.publish(arm_is_ok);
        pid_enable_pub.publish(pid_enable);
    }
}

void small_effort_callback(const std_msgs::Float64::ConstPtr& small_vel, Me_arm& myarm)
{
    if(arm_can_be_control && !targetPointArrivedFlag)
    {
        small_effort = small_vel->data;
        myarm.elmo_setVelocity(1, (int32_t)((small_vel->data) * 100));
    }
    else
    {
        myarm.elmo_setVelocity(1, 0);
        myarm.elmo_setVelocity(2, 0);
        ROS_ERROR("------------------SMALL stop");
        arm_is_ok.data = true;
        pid_enable.data = false;
        isok_pub.publish(arm_is_ok);
        pid_enable_pub.publish(pid_enable);
    }
}

void LR_effort_callback(const std_msgs::Float64::ConstPtr& LR_vel, Me_arm& myarm)
{
    LR_effort = LR_vel->data;
    if (LR_effort > 60.0)
        myarm.Stepper_run_withrelPosition(BIG_LR, (int32_t)(LR_effort), 1, 50);
    else if (LR_effort < -60.0)
        myarm.Stepper_run_withrelPosition(BIG_LR, (int32_t)(LR_effort), -1, 50);
    else
        myarm.Stepper_stop(BIG_LR);
}

void big_joint_callback(const std_msgs::Float64::ConstPtr& big_joint_deg_msg)
{
    big_joint_deg = big_joint_deg_msg->data;
}

void small_joint_callback(const std_msgs::Float64::ConstPtr& small_joint_deg_msg)
{
    small_joint_deg = small_joint_deg_msg->data;
}


void emergency_callback(const std_msgs::Bool::ConstPtr& msg, Me_arm& myarm)
{
    if(msg->data)
    {
        myarm.elmo_Disable(1);
        usleep(5000);
        myarm.elmo_Disable(2);
        myarm.Stepper_stop(OUTIN);
        myarm.Stepper_stop(BIG_LR);
    }
    else
    {
        myarm.elmo_enable(1);
        usleep(5000);
        myarm.elmo_enable(2);
    }
}

/* main */
int main(int argc, char* argv[])
{
    Me_arm myarm;
    double alpha_deg, beta_deg;//模型中的两个角
    geometry_msgs::Point current_point;

    arm_can_be_control = true;
    
    myarm.elmo_setmod(1, Velocity_mode);
    myarm.elmo_enable(1);

    myarm.elmo_setmod(2, Velocity_mode);
    myarm.elmo_enable(2);

    myarm.Stepper_stop(SCRAP);
    myarm.Stepper_stop(BIG_LR);
    sleep(1);

    // myarm.Stepper_run_withVelocity(SCRAP, -1, 200);

    // myarm.Stepper_run_withrelPosition(BIG_LR, (int32_t)(-5000), -1, 50);   
    ros::init(argc, argv, "arm_move_base");
    ros::NodeHandle nh /*("~")*/;

    ros::Publisher 	big_target_joint_deg_pub = nh.advertise<std_msgs::Float64>("/big_target_joint_deg", 1);
    ros::Publisher 	small_target_joint_deg_pub = nh.advertise<std_msgs::Float64>("/small_target_joint_deg", 1);
    ros::Publisher 	current_point_pub = nh.advertise<geometry_msgs::Point>("/current_point", 1);
    isok_pub = nh.advertise<std_msgs::Bool>("/arm_isok", 1);
    pid_enable_pub = nh.advertise<std_msgs::Bool>("/bs_pid_enable", 1);
    log_pub = nh.advertise<std_msgs::String>("/nvidia_log",20);
    

    ros::Subscriber vel_sub = nh.subscribe<arm_move::arm_motor_vel>("/arm_motor_vel", 1, boost::bind(&arm_callback, _1, myarm));
    ros::Subscriber big_effort_sub = nh.subscribe<std_msgs::Float64>("/big_control_effort", 1, boost::bind(&big_effort_callback, _1, myarm));
    ros::Subscriber small_effort_sub = nh.subscribe<std_msgs::Float64>("/small_control_effort", 1, boost::bind(&small_effort_callback, _1, myarm));
    ros::Subscriber LR_effort_sub = nh.subscribe<std_msgs::Float64>("/LR_effort", 1, boost::bind(&LR_effort_callback, _1, myarm));
    // ros::Subscriber joint_sub = nh.subscribe<robot_bringup::joint>("/joint_deg", 1, joint_callback);
    ros::Subscriber point_sub = nh.subscribe<geometry_msgs::Point>("/target_point", 10, boost::bind(&target_point_callback, _1, big_target_joint_deg_pub, small_target_joint_deg_pub));
    ros::Subscriber big_joint_sub = nh.subscribe<std_msgs::Float64>("/big_joint_deg", 10, big_joint_callback);
    ros::Subscriber small_joint_sub = nh.subscribe<std_msgs::Float64>("/small_joint_deg", 10, small_joint_callback);
    ros::Subscriber LR_run_to_abs_position_sub = nh.subscribe<std_msgs::Int32>("/LR_abs_position", 10, boost::bind(&LR_abs_positon_callback, _1, myarm));
    ros::Subscriber LR_run_to_rel_position_sub = nh.subscribe<std_msgs::Int32>("/LR_rel_position", 10, boost::bind(&LR_rela_positon_callback, _1, myarm));
    ros::Subscriber return_zero_sub = nh.subscribe<std_msgs::Bool>("/return_zero_sub", 10, boost::bind(&return_zero_callback, _1, myarm));
    emergency_sub = nh.subscribe<std_msgs::Bool>("/emergency_flag", 1, boost::bind(&emergency_callback, _1, myarm));
      
    ros::AsyncSpinner s(4);
    s.start();

    arm_is_ok.data = true;//默认机械臂空闲
    isok_pub.publish(arm_is_ok);//上传
    pid_enable.data = false;//默认关闭末端PID
    pid_enable_pub.publish(pid_enable);

    ros::Rate r(25);
    while (ros::ok())
    {
        /*计算两个当前末端位置 笛卡尔坐标系 原点在大臂转轴*/
        alpha_deg = 90.0f - big_joint_deg;
        beta_deg = alpha_deg - small_joint_deg - 90;
        current_point.x = L1 * cos(alpha_deg * pi / 180.0) + L2 * cos(beta_deg * pi / 180.0) + 0.035;
        current_point.z = L1 * sin(alpha_deg * pi / 180.0) + L2 * sin(beta_deg * pi / 180.0) - L3;
        current_point_pub.publish(current_point);

        if(!(arm_is_ok.data))
        {
            if (abs(targetXPoint - current_point.x) < 0.005f && abs(targetZPoint - current_point.z) < 0.005f)
            {
                targetPointArrivedFlag = true;
                arm_is_ok.data = true;
                isok_pub.publish(arm_is_ok);
            }
        }

        /*角度限位*/
        if(small_joint_deg < -48.0 || small_joint_deg > 16.0 || big_joint_deg > 63.0 || big_joint_deg < -8.0f)
        {
            arm_can_be_control = false;//false
        }
        else
            arm_can_be_control = true;
        r.sleep();
    }
}
