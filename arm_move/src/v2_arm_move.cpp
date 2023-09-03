 /*
 * @Author: Wei Zhifei
 * @Date: 2023-7-8 15:31:58
 * 未使用C++类的形式写，有C风格。全局变量较少。
 */

#include <unistd.h>
#include <iostream>
#include <cmath>
#include <thread>
#include <boost/thread.hpp>
#include <mutex>

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

std::mutex can_lock;

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

ros::Publisher 	big_pid_enable_pub;
ros::Publisher 	small_pid_enable_pub;
ros::Publisher 	small_vertical_pid_enable_pub;

ros::Publisher  isok_pub;
ros::Publisher  log_pub;
ros::Publisher  s_pub;
ros::Subscriber emergency_sub;

// ros::Publisher small_vertical_error_pub;

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

    std_msgs::Float64 big_deg, small_deg, s_msg;
    std_msgs::Int32 OUTIN_S;
    double S;

    targetXPoint = target_msg->x;
    targetZPoint = target_msg->z;

    // bool res = get_started(target_msg->x - 0.035, target_msg->z, target_radian);

    big_deg.data = asin((targetZPoint + L2) / L1);
    S = - L1 * ( cos(big_deg.data) - cos(big_joint_deg * pi / 180));
    big_deg.data = big_deg.data * 180 / pi;

    s_msg.data = S;
    s_pub.publish(s_msg);
    
    ROS_INFO("big: %f deg ------ S : %f cm ", big_deg.data, S);
    if((big_deg.data > 72.0) || (big_deg.data < 0.0))
    {
        ROS_ERROR("big: %f deg", big_deg.data);
        log_to_pc("[arm_move] error : joint_deg is exceed threshold!");
    }
    else
    {
        big_pub.publish(big_deg);   // alpha and beta degree
        // small_pub.publish(small_deg);

        pid_enable.data = true;
        big_pid_enable_pub.publish(pid_enable); //打开大臂pid
        small_vertical_pid_enable_pub.publish(pid_enable); //打开小臂垂直pid

        ROS_INFO("big: %f deg", big_deg.data);
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

void arm_outin_callback(const std_msgs::Int32::ConstPtr& motor_vel, Me_arm& myarm)
{
    if (motor_vel->data > 0)
        myarm.Stepper_run_withVelocity(OUTIN, 1, (float_t)(motor_vel->data));
    else if (motor_vel->data < 0)
        myarm.Stepper_run_withVelocity(OUTIN, -1, (float_t)(-(motor_vel->data)));
    else
        myarm.Stepper_stop(OUTIN);
}

void return_zero_callback(const std_msgs::BoolConstPtr &return_msg, Me_arm &myarm)
{
    if(return_msg->data)
    {
        myarm.Stepper_findzeroPosition(BIG_LR);//回零
        myarm.Stepper_findzeroPosition(OUTIN);//回零
    }
}

void LR_abs_positon_callback(const std_msgs::Int32ConstPtr &pos_msg, Me_arm &myarm)
{
    std::unique_lock<std::mutex> lg(can_lock);
    myarm.Stepper_run_withabsPosition(BIG_LR, pos_msg->data, 100);
    lg.unlock();
}

void LR_rela_positon_callback(const std_msgs::Int32ConstPtr& rela_msg, Me_arm& myarm)
{
    std::unique_lock<std::mutex> lg(can_lock);
    if (rela_msg->data > 0)
        myarm.Stepper_run_withrelPosition(BIG_LR, rela_msg->data,  1, 100);
    else if (rela_msg->data < 0)
        myarm.Stepper_run_withrelPosition(BIG_LR, rela_msg->data, -1, 100);
    else
        myarm.Stepper_stop(BIG_LR);
    lg.unlock();
}

void OUTIN_abs_positon_callback(const std_msgs::Int32ConstPtr &pos_msg, Me_arm &myarm)
{
    std::unique_lock<std::mutex> lg(can_lock);
    myarm.Stepper_run_withabsPosition(OUTIN, pos_msg->data, 100);
    lg.unlock();
}

void OUTIN_rela_positon_callback(const std_msgs::Int32ConstPtr& rela_msg, Me_arm& myarm)
{
    std::unique_lock<std::mutex> lg(can_lock);
    if (rela_msg->data > 0)
        myarm.Stepper_run_withrelPosition(OUTIN, rela_msg->data,  1, 100);
    else if (rela_msg->data < 0)
        myarm.Stepper_run_withrelPosition(OUTIN, rela_msg->data, -1, 100);
    else
        myarm.Stepper_stop(OUTIN);
    lg.unlock();
}

void big_effort_callback(const std_msgs::Float64::ConstPtr& big_vel, Me_arm& myarm)
{
    std::unique_lock<std::mutex> lg(can_lock);
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
        big_pid_enable_pub.publish(pid_enable);
        small_vertical_pid_enable_pub.publish(pid_enable);
    }
    can_lock.unlock();
}

void small_effort_callback(const std_msgs::Float64::ConstPtr& small_vel, Me_arm& myarm)
{
    std::unique_lock<std::mutex> lg(can_lock);
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
        big_pid_enable_pub.publish(pid_enable);
        small_vertical_pid_enable_pub.publish(pid_enable);
    }
    can_lock.unlock();
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

int outin_motor_status = 0;
int lr_motor_status = 0;

// void checkstatus_thread(Me_arm _myarm)
// {
//     ros::Rate r(10);
//     while(ros::ok())
//     {
//         std::unique_lock<std::mutex> lg(can_lock);
//         _myarm.Stepper_checkStatus(OUTIN);
//         outin_motor_status = _myarm.outin_motor.motor_running_status;
//         std::cout << "OUTIN_Status: " << _myarm.outin_motor.motor_running_status << std::endl;
//         lg.unlock();
//         usleep(500000);
//         std::unique_lock<std::mutex> lg1(can_lock);
//         _myarm.Stepper_checkStatus(BIG_LR);
//         lr_motor_status = _myarm.lr_motor.motor_running_status;
//         std::cout << "LR_Status: " << _myarm.lr_motor.motor_running_status << std::endl;
//         lg1.unlock();
//         r.sleep();
//     }
//     return;
// }

/* main */
int main(int argc, char* argv[])
{
    Me_arm myarm;
    geometry_msgs::Point current_point;

    arm_can_be_control = true;
    
    myarm.elmo_setmod(1, Velocity_mode);
    myarm.elmo_enable(1);

    myarm.elmo_setmod(2, Velocity_mode);
    myarm.elmo_enable(2);

    myarm.Stepper_stop(SCRAP);
    myarm.Stepper_stop(BIG_LR);
    myarm.Stepper_stop(OUTIN);
    sleep(1);

    // myarm.Stepper_run_withVelocity(SCRAP, -1, 200);

    // myarm.Stepper_run_withrelPosition(BIG_LR, (int32_t)(-5000), -1, 50);   
    ros::init(argc, argv, "arm_move_base");
    ros::NodeHandle nh /*("~")*/;

    ros::Publisher 	big_target_joint_deg_pub = nh.advertise<std_msgs::Float64>("/big_target_joint_deg", 1);
    ros::Publisher 	small_target_joint_deg_pub = nh.advertise<std_msgs::Float64>("/small_target_joint_deg", 1);
    ros::Publisher 	current_point_pub = nh.advertise<geometry_msgs::Point>("/current_point", 1);
    isok_pub = nh.advertise<std_msgs::Bool>("/arm_isok", 1);
    big_pid_enable_pub = nh.advertise<std_msgs::Bool>("/big_pid_enable", 1);
    small_pid_enable_pub = nh.advertise<std_msgs::Bool>("/small_pid_enable", 1);
    small_vertical_pid_enable_pub = nh.advertise<std_msgs::Bool>("/small_vertical_pid_enable", 1);
    log_pub = nh.advertise<std_msgs::String>("/nvidia_log",20);
    s_pub = nh.advertise<std_msgs::Float64>("/s_distance", 1);

    ros::Subscriber vel_sub = nh.subscribe<arm_move::arm_motor_vel>("/arm_motor_vel", 1, boost::bind(&arm_callback, _1, myarm));
    ros::Subscriber out_vel_sub = nh.subscribe<std_msgs::Int32>("/arm_motor_vel_outin", 1, boost::bind(&arm_outin_callback, _1, myarm));
    ros::Subscriber big_effort_sub = nh.subscribe<std_msgs::Float64>("/big_control_effort", 1, boost::bind(&big_effort_callback, _1, myarm));
    ros::Subscriber small_effort_sub = nh.subscribe<std_msgs::Float64>("/small_control_effort", 1, boost::bind(&small_effort_callback, _1, myarm));
    ros::Subscriber LR_effort_sub = nh.subscribe<std_msgs::Float64>("/LR_effort", 1, boost::bind(&LR_effort_callback, _1, myarm));
    ros::Subscriber point_sub = nh.subscribe<geometry_msgs::Point>("/target_point", 10, boost::bind(&target_point_callback, _1, big_target_joint_deg_pub, small_target_joint_deg_pub));
    ros::Subscriber big_joint_sub = nh.subscribe<std_msgs::Float64>("/big_joint_deg", 10, big_joint_callback);
    ros::Subscriber small_joint_sub = nh.subscribe<std_msgs::Float64>("/small_joint_deg", 10, small_joint_callback);
    ros::Subscriber LR_run_to_abs_position_sub = nh.subscribe<std_msgs::Int32>("/LR_abs_position", 10, boost::bind(&LR_abs_positon_callback, _1, myarm));
    ros::Subscriber LR_run_to_rel_position_sub = nh.subscribe<std_msgs::Int32>("/LR_rel_position", 10, boost::bind(&LR_rela_positon_callback, _1, myarm));
    ros::Subscriber OUTIN_run_to_abs_position_sub = nh.subscribe<std_msgs::Int32>("/OUTIN_abs_position", 10, boost::bind(&OUTIN_abs_positon_callback, _1, myarm));
    ros::Subscriber OUTIN_run_to_rel_position_sub = nh.subscribe<std_msgs::Int32>("/OUTIN_rel_position", 10, boost::bind(&OUTIN_rela_positon_callback, _1, myarm));
    ros::Subscriber return_zero_sub = nh.subscribe<std_msgs::Bool>("/return_zero_sub", 10, boost::bind(&return_zero_callback, _1, myarm));
    emergency_sub = nh.subscribe<std_msgs::Bool>("/emergency_flag", 1, boost::bind(&emergency_callback, _1, myarm));
      
    ros::AsyncSpinner s(4);
    s.start();

    // std::thread checkStatusThread(checkstatus_thread, myarm);
    // checkStatusThread.detach();

    arm_is_ok.data = true;//默认机械臂空闲
    isok_pub.publish(arm_is_ok);//上传
    pid_enable.data = false;//默认关闭末端PID
    big_pid_enable_pub.publish(pid_enable);
    small_vertical_pid_enable_pub.publish(pid_enable);

    ros::Rate r(25);
    while (ros::ok())
    {
        /*计算两个当前末端位置 笛卡尔坐标系 原点在大臂转轴*/
        current_point.x = L1 * cos(big_joint_deg * pi / 180.0) + 0.035;
        current_point.z = L1 * sin(big_joint_deg * pi / 180.0) - L2;
        current_point_pub.publish(current_point);

        if(!(arm_is_ok.data))
        {
            // printf("-------motor_status--------%d\n",outin_motor_status);
            if ((abs(big_joint_deg - small_joint_deg) < 0.5) \
                && (lr_motor_status == Me_arm::Stepper_Running_Status::FREE)\
                && (outin_motor_status == Me_arm::Stepper_Running_Status::FREE) \
                && abs(targetZPoint - current_point.z) < 0.008f)
            {
                targetPointArrivedFlag = true;
                arm_is_ok.data = true;
                isok_pub.publish(arm_is_ok);
            }
        }

        /*角度限位*/
        if(big_joint_deg > 72.0 || big_joint_deg < 0.0f)
        {
            arm_can_be_control = false;//false
        }
        else
            arm_can_be_control = true;
        r.sleep();
    }
}
