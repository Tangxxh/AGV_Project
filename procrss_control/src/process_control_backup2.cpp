/*
 * @Author: Wei Zhifei
 * @Date: 2023-2-22 13:18:18
 * @desp: 此文件使用C语言风格，用C++类进行封装，主要进行作业流程控制，大部分使用全局变量进行数据传递
 */

#include <unistd.h>
#include <iostream>
#include <signal.h>
#include <sys/wait.h>
#include <string>
#include <vector>

#include <ros/ros.h>

#include <geometry_msgs/Point.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

#include <arm_move/arm_motor_vel.h>

enum baseProcessNameList
{
    //这里填写要启动的子进程
    robot_bringup,
    arm_move_base,

    //这里是主线程 不要更改
    main_process
};

// enum navProcessNameList
// {
//     //这里填写要启动的子进程

//     robot_bringup,
//     arm_move_base,

//     //这里是主线程 不要更改
//     main_process
// };

typedef struct allPidList
{
    std::string nodename;
    pid_t pid;
}allPID_t;

typedef struct allPkgLaunchList
{
    std::string pkgname;
    std::string launchname;
    pid_t pid;
}process_t;

class Process_Control
{
private:

    bool arm_is_ok;   // 自动控制结束标志，true表示可以手动控制
    double x_distance, tof_distance, x_real_error;
    std::vector<allPID_t> pidAllList;
    std::vector<process_t> pkgLaunchAllList;

    ros::Publisher abs_pub;
    ros::Publisher target_point_pub;
    ros::Publisher LR_target_pub;
    ros::Publisher LR_control_enable_pub;
    ros::Publisher arm_motion_pub;
    ros::Publisher log_pub;

    ros::Subscriber isok_sub;
    ros::Subscriber x_distance_sub;
    ros::Subscriber current_point_sub;
    ros::Subscriber tof_distance_sub;
    ros::Subscriber x_real_error_sub;

    geometry_msgs::Point point_buf;
    std_msgs::Float64 data_buf;
    std_msgs::Bool state_buf;
    std_msgs::Int32 int_buf;
    arm_move::arm_motor_vel arm_buf;

    geometry_msgs::Point current_point;
    ros::NodeHandle nh;

public:
    
    Process_Control()
    {
        arm_is_ok  = true;

        abs_pub = nh.advertise<std_msgs::Int32>("/LR_abs_position", 1);
        target_point_pub = nh.advertise<geometry_msgs::Point>("/target_point", 1);
        LR_target_pub = nh.advertise<std_msgs::Float64>("/mid_set_point", 1);
        LR_control_enable_pub = nh.advertise<std_msgs::Bool>("/LR_pid_enable", 1);
        arm_motion_pub = nh.advertise<arm_move::arm_motor_vel>("/arm_motor_vel", 1);
        log_pub = nh.advertise<std_msgs::String>("/nvidia_log",20);

        isok_sub = nh.subscribe<std_msgs::Bool>("/arm_isok", 10, &Process_Control::isOkCallback, this, ros::TransportHints().tcpNoDelay());
        x_distance_sub = nh.subscribe<std_msgs::Float64>("/x_distance", 10, &Process_Control::x_distanceCallback, this, ros::TransportHints().tcpNoDelay());
        current_point_sub = nh.subscribe<geometry_msgs::Point>("/current_point", 10, &Process_Control::current_pointCallback, this, ros::TransportHints().tcpNoDelay());
        tof_distance_sub = nh.subscribe<std_msgs::Float64>("/tof_distance", 10, &Process_Control::tof_distance_callback, this, ros::TransportHints().tcpNoDelay());
        x_real_error_sub = nh.subscribe<std_msgs::Float64>("/x_real_error", 10,&Process_Control::x_real_error_callback, this, ros::TransportHints().tcpNoDelay());

        state_buf.data = true;
        LR_control_enable_pub.publish(state_buf);// 打开PID控制器
    }
    ~Process_Control(){}

    int scrap_process(void)
    {
        int_buf.data = 76000;
        abs_pub.publish(int_buf);//横向运行到中间

        ROS_INFO("RUN THIS !!!!!!``");
        // while (!arm_is_ok);//等待上次占用结束
        point_buf.x = 0.59; // x_distance;
        point_buf.z = -0.25;
        target_point_pub.publish(point_buf);//初步对准x，并下放夹爪
        ROS_INFO("culue down 55 20");
        sleep(17);
        /*进入PID调中控制*/
        data_buf.data = 0.0;
        LR_target_pub.publish(data_buf);
        state_buf.data = true;
        LR_control_enable_pub.publish(state_buf);// 打开PID控制器

        /*测量z轴高度差*/
        sleep(8);
        state_buf.data = false;
        LR_control_enable_pub.publish(state_buf);// guanPID控制器
        sleep(2);
        /*精准下放夹爪到15cm高度*/
        point_buf.z = current_point.z - tof_distance/1000;
        target_point_pub.publish(point_buf);//下放夹爪
        ROS_INFO("z_distance = %f", tof_distance);
        sleep(8);
       
        ROS_INFO("X_ERROR = %f", x_real_error/100);
        point_buf.x = current_point.x + x_real_error/100;
        point_buf.z = current_point.z - 0.10;
        // while (!arm_is_ok);//等待上次占用结束
        ROS_INFO("x = %f  |  z = %f", point_buf.x, point_buf.z);
        target_point_pub.publish(point_buf);//下放夹爪
                
        sleep(10);//
        /*抓取*/
        // while (!arm_is_ok);//等待上次占用结束
        arm_buf.stepper_two = -1;//正数为抓取
        arm_buf.almo_one = 0;
        arm_buf.almo_two = 0;
        arm_buf.stepper_one = 0;
        arm_motion_pub.publish(arm_buf);
        sleep(6);
        point_buf.x = 0.45;
        point_buf.z = -0.05;
        // while (!arm_is_ok);//等待上次占用结束
        ROS_INFO("x = %f  |  z = %f", point_buf.x, point_buf.z);
        target_point_pub.publish(point_buf);//下放夹爪

        int_buf.data = 0;
        abs_pub.publish(int_buf);//横向运行到中间

        sleep(10);

        arm_buf.stepper_two = 1;//正数为抓取
        arm_buf.almo_one = 0;
        arm_buf.almo_two = 0;
        arm_buf.stepper_one = 0;
        arm_motion_pub.publish(arm_buf);
        
        return 0;
    }

    /*保留函数，供双目测距使用*/
    geometry_msgs::Point cal_point_by_vision(int _a, int _b)
    {
        geometry_msgs::Point point_in_scrap;
        return point_in_scrap;
    }
    
    int baseProcessStart(void)
    {
        // std::cout << pidList.at(0).nodename << " pid:" << pidList[0].pid << std::endl \
        //     << pidList.at(1).nodename << " pid:" << pidList[1].pid << std::endl ;

        int i, j, status, ret;
        pid_t pid;
        for (i = 0; i < sizeof(baseProcessNameList)/sizeof(int) ; i++)
        {
            pid = fork();
            if(pid ==0)
                break;
            else if(pid < 0)
            {
                std::cout << "fork error" << std::endl;
            }
            else
            {
                allPID_t listbuf;
                switch(i)
                {
                    case robot_bringup:
                        listbuf.pid = pid;
                        listbuf.nodename = "robot_bringup";
                        pidAllList.push_back(listbuf);
                        break;
                    case arm_move_base:
                        listbuf.pid = pid;
                        listbuf.nodename = "arm_move_base";
                        pidAllList.push_back(listbuf);
                        break;
                }
            }
        }
        switch(i)
        {
            case robot_bringup:
                std::cout << "this is robot_bringup process ,pid is " << '[' << getpid() << ']' << std::endl; 
                execlp("roslaunch", "roslaunch", "robot_bringup", "bringup.launch", NULL);
                std::cout << "run this1 ----------------------------" << std::endl;
                exit(0);
                break;
            case arm_move_base:
                std::cout << "this is arm_move_pid process ,pid is " << '[' << getpid() << ']' << std::endl; 
                execlp("roslaunch", "roslaunch", "arm_move", "arm_move.launch", NULL);
                std::cout << "run this2 ----------------------------" << std::endl;
                exit(0);
                break;
            case main_process:
                // std::cout << pidList.at(0).nodename << " pid:" << pidList[0].pid << std::endl \
                //     << pidList.at(1).nodename << " pid:" << pidList[1].pid << std::endl ;
                break;
            default:
                break;         
        }
        return 0;
    }

    int processStart(std::vector<process_t> &startList)
    {
        int i, j, status, ret;
        pid_t pid;
        for (i = 0; i < startList.size(); i++)
        {
            pid = fork();
            if(pid ==0)
                break;
            else if(pid < 0)
            {
                std::cout << "fork error" << std::endl;
                return -1;
            }
            else
            {
                startList[i].pid = pid;
                pkgLaunchAllList.push_back(startList[i]);
            }
        }

        if(i < startList.size())
        {
            for(j = 0; j < startList.size(); j++)
            {
                if(i == j)
                {
                    std::cout << "this is "<< startList[i].pkgname << "process ,pid is " << '[' << getpid() << ']' << std::endl; 
                    execlp("roslaunch", "roslaunch", startList[i].pkgname.c_str(), startList[i].launchname.c_str(), NULL);
                    std::cout << "run this1 ----------------------------" << std::endl;
                    exit(0);                
                }
            }
        }

        std::cout   << "------------------this is main process--------------------" << std::endl \
                    << pkgLaunchAllList.at(0).pkgname << " pid:" << pkgLaunchAllList[0].pid << std::endl \
                    << pkgLaunchAllList.at(1).pkgname << " pid:" << pkgLaunchAllList[1].pid << std::endl ;

        return 0;
    }

    int closeAllProcess(std::vector<allPID_t> &pidList)
    {
        int ret, status, pid;

        for(int i = 0; i< pidList.size(); i++)
        {
            ret = kill(pidList[i].pid, SIGINT);
            pid = wait(&status);
            if(0 == ret)
                ROS_INFO("%s is killed successfully. PID[%d]", pidList[i].nodename.c_str(), pid);   
            else
            {
                ROS_ERROR("%s killing failed.", pidList[i].nodename.c_str());
                return -1;
            }
        }
        return 0;     
    }

    void log_to_pc(const char* log)
    {
        std_msgs::String logPC;
        logPC.data = log;
        this->log_pub.publish(logPC);
    }

    void isOkCallback(const std_msgs::BoolConstPtr &ok_msgs)
    {
        arm_is_ok = ok_msgs->data;
    }

    void x_distanceCallback(const std_msgs::Float64ConstPtr& x_dis_msg)
    {
        x_distance = x_dis_msg->data;
    }

    void tof_distance_callback(const std_msgs::Float64ConstPtr& tof_msg)
    {
        tof_distance = tof_msg->data;
    }

    void x_real_error_callback(const std_msgs::Float64ConstPtr& x_error_msg)
    {
        x_real_error = x_error_msg->data;
    }

    void current_pointCallback(const geometry_msgs::PointConstPtr &point_msg)
    {
        current_point.x = point_msg->x;
        current_point.z = point_msg->z;
    }
};

/* main */
int main(int argc, char* argv[])
{
    int status,ret;

    ros::init(argc, argv, "process_control");
    Process_Control PC;
    ros::AsyncSpinner s(4);
    s.start();

    std::vector<process_t> startList = 
    {
        {"robot_bringup", "bringup.launch", 0},
        {"arm_move", "arm_move.launch", 0}
    };
    PC.processStart(startList);
    // sleep(5);
    // PC.closeAllProcess();

    ros::Rate r(1);
    int i = 0;
    while (ros::ok())
    {    
        PC.log_to_pc("test");
        r.sleep();
        
        // sleep(10);
        // PC.scrap_process();
        // r.sleep();
        // break;
    }
    
    return 0;
}
//