/*
 * @Author: Wei Zhifei
 * @Date: 2023-2-22 13:18:18
 * @desp: 此文件使用C语言风格，主要进行作业流程控制，大部分使用全局变量进行数据传递
 */

#include <unistd.h>
#include <iostream>

#include <ros/ros.h>

#include <geometry_msgs/Point.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>

#include <arm_move/arm_motor_vel.h>

bool arm_is_ok;   // 自动控制结束标志，true表示可以手动控制
double x_distance, z_distance;
ros::Publisher abs_pub;
ros::Publisher target_point_pub;
ros::Publisher LR_target_pub;
ros::Publisher LR_control_enable_pub;
ros::Publisher arm_motion_pub;

geometry_msgs::Point point_buf;
std_msgs::Float64 data_buf;
std_msgs::Bool state_buf;
std_msgs::Int32 int_buf;
arm_move::arm_motor_vel arm_buf;

geometry_msgs::Point current_point;

int scrap_process(void)
{
    int_buf.data = 76000;
    abs_pub.publish(int_buf);//横向运行到中间

    while (!arm_is_ok);//等待上次占用结束
    point_buf.x = x_distance;
    point_buf.z = -0.35;
    target_point_pub.publish(point_buf);//初步对准x，并下放夹爪

    /*进入PID调中控制*/
    data_buf.data = 0.0;
    LR_target_pub.publish(data_buf);
    state_buf.data = true;
    LR_control_enable_pub.publish(state_buf);// 打开PID控制器

    /*测量z轴高度差*/

    /*精准下放夹爪*/
    point_buf.z = current_point.z - z_distance;
    while (!arm_is_ok);//等待上次占用结束
    target_point_pub.publish(point_buf);//下放夹爪

    /*抓取*/
    while (!arm_is_ok);//等待上次占用结束
    arm_buf.stepper_two = 1;//正数为抓取
    arm_buf.almo_one = 0;
    arm_buf.almo_two = 0;
    arm_buf.stepper_one = 0;
    arm_motion_pub.publish(arm_buf);

    return 0;
}

void isOkCallback(const std_msgs::BoolConstPtr &ok_msgs)
{
    arm_is_ok = ok_msgs->data;
}

void x_distanceCallback(const std_msgs::Float64ConstPtr& x_dis_msg)
{
    x_distance = x_dis_msg->data;
}

void z_distanceCallback(const std_msgs::Float64ConstPtr& z_dis_msg)
{
    z_distance = z_dis_msg->data;
}

void current_pointCallback(const geometry_msgs::PointConstPtr &point_msg)
{
    current_point.x = point_msg->x;
    current_point.z = point_msg->z;
}
/* main */
int main(int argc, char* argv[])
{
    ros::init(argc, argv, "arm_move_base");
    ros::NodeHandle nh /*("~")*/;

    // ros::Publisher 	big_target_joint_deg_pub = nh.advertise<std_msgs::Float64>("/big_target_joint_deg", 1);
    // ros::Publisher 	small_target_joint_deg_pub = nh.advertise<std_msgs::Float64>("/small_target_joint_deg", 1);
    // ros::Publisher 	current_point_pub = nh.advertise<geometry_msgs::Point>("/current_point", 1);
    // ros::Publisher 	isok_pub = nh.advertise<std_msgs::Bool>("/arm_isok", 1);
    abs_pub = nh.advertise<std_msgs::Int32>("/LR_abs_position",1);
    target_point_pub = nh.advertise<geometry_msgs::Point>("/target_point", 1);
    LR_target_pub = nh.advertise<std_msgs::Float64>("/mid_set_point",1);
    LR_control_enable_pub = nh.advertise<std_msgs::Bool>("/LR_pid_enable",1);
    arm_motion_pub = nh.advertise<arm_move::arm_motor_vel>("/arm_motor_vel", 1);
    // ros::Subscriber vel_sub = nh.subscribe<arm_move::arm_motor_vel>("/arm_motor_vel", 1, boost::bind(&arm_callback, _1, myarm));
    // ros::Subscriber big_effort_sub = nh.subscribe<std_msgs::Float64>("/big_control_effort", 1, boost::bind(&big_effort_callback, _1, myarm));
    // ros::Subscriber small_effort_sub = nh.subscribe<std_msgs::Float64>("/small_control_effort", 1, boost::bind(&small_effort_callback, _1, myarm));
    // // ros::Subscriber joint_sub = nh.subscribe<robot_bringup::joint>("/joint_deg", 1, joint_callback);
    // ros::Subscriber point_sub = nh.subscribe<geometry_msgs::Point>("/target_point", 10, boost::bind(&target_point_callback, _1, big_target_joint_deg_pub, small_target_joint_deg_pub));
    // ros::Subscriber big_joint_sub = nh.subscribe<std_msgs::Float64>("/big_joint_deg", 10, boost::bind(&big_joint_callback, _1, big_joint_deg));
    // ros::Subscriber small_joint_sub = nh.subscribe<std_msgs::Float64>("/small_joint_deg", 10, boost::bind(&small_joint_callback, _1, small_joint_deg));
    ros::Subscriber isok_sub = nh.subscribe<std_msgs::Bool>("/arm_isok", 1, isOkCallback);
    ros::Subscriber x_distance_sub = nh.subscribe<std_msgs::Float64>("/x_distance", 1, x_distanceCallback);
    ros::Subscriber z_distance_sub = nh.subscribe<std_msgs::Float64>("/z_distance", 1, z_distanceCallback);
    ros::Subscriber current_point_sub = nh.subscribe<geometry_msgs::Point>("/current_point", 1, current_pointCallback);

    ros::AsyncSpinner s(4);
    s.start();

    ros::Rate r(5);
    while (ros::ok())
    {
        scrap_process();
        r.sleep();
    }
    
    return 0;
}
