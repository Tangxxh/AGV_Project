#include <ros/ros.h>

#include <robot_bringup/vel.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>

#include <iostream>

#include "pid.hpp"

class LineFollow
{
private:
    ros::NodeHandle _nh;
    ros::Publisher _pub_singleVel;

    ros::Subscriber _sub_lineOffset;
    ros::Subscriber _sub_runFlag;
    ros::Subscriber _sub_target_vel;
    pid PIDControler;

    int32_t target_vel;
    bool run_flag;

public:
    LineFollow()
    {
        _sub_lineOffset  = _nh.subscribe("/line_offset", 1, &LineFollow::offsetControlCB, this, ros::TransportHints().tcpNoDelay());
        _sub_runFlag = _nh.subscribe("/line_run_flag", 1 , &LineFollow::runFlagCB, this, ros::TransportHints().tcpNoDelay());
        _sub_target_vel = _nh.subscribe("/pid_target_vel", 1 , &LineFollow::targetVelCB, this, ros::TransportHints().tcpNoDelay());
        _pub_singleVel = _nh.advertise<robot_bringup::vel>("/single_vel",1);

        run_flag = false;
        target_vel = 100;
    }
    ~LineFollow(){}

    void offsetControlCB(const std_msgs::Int32::ConstPtr& offsetMsg)
    {
        if(run_flag)
        {
            speedOutStr pidout;
            robot_bringup::vel single_vel;
            //ROS_INFO("RECEIVE OFFSET!");
            pidout = PIDControler.Robot_PID_Ctrl(-target_vel,offsetMsg->data);//
            single_vel.vel_left = pidout.value_left;
            single_vel.vel_right = pidout.value_right;
            // ROS_INFO("LEFT =  %d | RIGHT = %d ",single_vel.vel_left,single_vel.vel_right);
            _pub_singleVel.publish(single_vel);    
        }
    } 

    void runFlagCB(const std_msgs::Bool::ConstPtr &msg)
    {
        run_flag = msg->data;
        robot_bringup::vel single_vel;
        single_vel.vel_left = 0;
        single_vel.vel_right = 0;  
        _pub_singleVel.publish(single_vel);  
    }   

    void targetVelCB(const std_msgs::Int32::ConstPtr &msg)
    {
        target_vel = msg->data;
    }
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "line_follow");
    LineFollow LF;

    ROS_INFO("Waiting for line_run_flag");
    ros::AsyncSpinner spinner(2); // Use multi threads
    spinner.start();
    ros::waitForShutdown();
    return 0;
}












