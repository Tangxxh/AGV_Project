/*
 * @Author: Wei Zhifei
 * @Date: 2022-11-12 13:18:38
 */
#include <ros/ros.h>
#include "utilty.h"
#include <robot_bringup/classis_status.h>
#include <nav_msgs/Odometry.h>
#include "serialPort.hpp"

extern double theta;

namespace Aimo
{
    
};

namespace Classis
{
    int analy_data(CANTxMsg_t *msg,robot_bringup::classis_status::Ptr ptrTopicMsg, nav_msgs::Odometry::Ptr odom, double wheelbase);//
};

namespace LeiSai
{

};

namespace Laser
{
    double analy_data(serialPort &myserial);//会阻塞线程
};

namespace Mark
{
    int analy_data(serialPort &myserial, std::string str, int length);
}