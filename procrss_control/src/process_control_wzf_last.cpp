/*
 * @Author: Wei Zhifei
 * @Date: 2023-2-22 13:18:18
 * @desp: 此文件使用C语言风格，用C++类进行封装，主要进行作业流程控制，大部分使用类内私有变量进行数据传�? */

#include <unistd.h>
#include <iostream>
#include <signal.h>
#include <sys/wait.h>
#include <string>
#include <vector>
#include <thread>
#include <stdio.h>
// #include <conio.h>
#include <pthread.h>

#include <ros/ros.h>

#include <geometry_msgs/Point.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>

#include <arm_move/arm_motor_vel.h>
#include <robot_bringup/get_tof_distance.h>



/*存放启动后节点的class*/
class allPkgLaunchList
{
public:
    allPkgLaunchList(std::string _pkgname, std::string _launchname, pid_t _pid)
    {
        this->pkgname = _pkgname;
        this->launchname = _launchname;
        this->pid = _pid;
    }
    ~allPkgLaunchList(){}
    std::string pkgname;
    std::string launchname;
    pid_t pid; 

    bool operator==(allPkgLaunchList t){
        return t.pkgname == this->pkgname && t.launchname == this->launchname;
    }
};

class Process_Control
{
private:

    bool arm_is_ok;   // 自动控制结束标志，true表示可以手动控制
    bool _taskFlag;   //开启自动任务标志位
    bool _initFlag;
    bool _trackingCompletedFlag;//巡线结束标志
    // bool _midPositionReachedFlag;
    double x_distance, tof_distance, x_real_error;
    double _side_pixel_error;   //侧面相机前进方向像素误差
    double roll,pitch,yaw; //欧拉角，单位弧度
    int tf_flag;
    uint32_t _num_label;
    int32_t mark_state;
    std::vector<allPkgLaunchList> pkgLaunchAllList;
    std::thread *staticTFBroadcasterThread;
    float ultrasonicDistance;
    float ultrasonicDistanceThreshold;
    bool rtk_aviliable;

    ros::Publisher abs_pub;
    ros::Publisher target_point_pub;
    ros::Publisher LR_target_pub;
    ros::Publisher LR_control_enable_pub;
    ros::Publisher arm_motion_pub;
    ros::Publisher log_pub;
    ros::Publisher line_run_pub;
    ros::Publisher side_camera_run_pub;
    ros::Publisher pid_targrt_vel_pub;
    ros::Publisher rtk_run_pub;
    ros::Publisher odom_run_pub;
    ros::Publisher track_path_switch_pub;
    ros::Publisher mark_pub;
    ros::Publisher cmd_vel_pub;
    ros::Publisher big_target_joint_deg_pub;
    ros::Publisher small_target_joint_deg_pub;
    ros::Publisher pid_enable_pub;

    ros::Subscriber isok_sub;
    ros::Subscriber x_distance_sub;
    ros::Subscriber current_point_sub;
    ros::Subscriber tof_distance_sub;
    ros::Subscriber x_real_error_sub;
    ros::Subscriber side_pixel_error_sub;

    ros::Subscriber startInitSub;
    ros::Subscriber startTrackingSub;
    ros::Subscriber startLineFollowSub;
    ros::Subscriber startScrapSub;
    ros::Subscriber startTaskSub;
    ros::Subscriber trackingCompletedSub;
    ros::Subscriber odometrySubscriber;
    ros::Subscriber numLabelSubscriber;
    ros::Subscriber markCompletedSub;
    ros::Subscriber imuOdomSub;
    ros::Subscriber ultraSonicSub;
    ros::Subscriber navFixSub;

    ros::ServiceClient _getTofDistance;

    geometry_msgs::PoseStamped _currentPose;
    geometry_msgs::Point point_buf;
    
    std_msgs::Float64 data_buf;
    std_msgs::Bool state_buf;
    std_msgs::Int32 int_buf;
    arm_move::arm_motor_vel arm_buf;
    geometry_msgs::Point current_point;

    // 创建 tf2 的广播   
    tf2_ros::StaticTransformBroadcaster static_broadcaster;

    ros::NodeHandle nh;

public:
    
    Process_Control()
    {
        arm_is_ok  = true;
        _taskFlag = false;
        // _midPositionReachedFlag = false;
        _initFlag = false;
        _trackingCompletedFlag = false;
        _side_pixel_error = 9999.99;
        _num_label = 0;
        mark_state = 0;
        tf_flag = 0;
        ultrasonicDistance = 0.0;
        ultrasonicDistanceThreshold = 0.0;
        rtk_aviliable = false;

        abs_pub = nh.advertise<std_msgs::Int32>("/LR_abs_position", 1);
        target_point_pub = nh.advertise<geometry_msgs::Point>("/target_point", 1);
        LR_target_pub = nh.advertise<std_msgs::Float64>("/mid_set_point", 1);
        LR_control_enable_pub = nh.advertise<std_msgs::Bool>("/LR_pid_enable", 1);
        arm_motion_pub = nh.advertise<arm_move::arm_motor_vel>("/arm_motor_vel", 1);
        log_pub = nh.advertise<std_msgs::String>("/nvidia_log",20);
        line_run_pub = nh.advertise<std_msgs::Bool>("/line_run_flag",1);
        side_camera_run_pub = nh.advertise<std_msgs::Bool>("/side_camera_run_flag",1);
        rtk_run_pub = nh.advertise<std_msgs::Bool>("/rtk_run_flag",1);
        odom_run_pub = nh.advertise<std_msgs::Bool>("/odom_run_flag", 1);
        pid_targrt_vel_pub = nh.advertise<std_msgs::Int32>("/pid_target_vel", 1);
        track_path_switch_pub = nh.advertise<std_msgs::Int32>("/track_path_switch", 1);
        mark_pub = nh.advertise<std_msgs::Int16>("/mark", 1);
        cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        big_target_joint_deg_pub = nh.advertise<std_msgs::Float64>("/big_target_joint_deg", 1);
        small_target_joint_deg_pub = nh.advertise<std_msgs::Float64>("/small_target_joint_deg", 1);
        pid_enable_pub = nh.advertise<std_msgs::Bool>("/bs_pid_enable", 1);

        isok_sub = nh.subscribe<std_msgs::Bool>("/arm_isok", 10, &Process_Control::isOkCallback, this, ros::TransportHints().tcpNoDelay());
        x_distance_sub = nh.subscribe<std_msgs::Float64>("/x_distance", 10, &Process_Control::x_distanceCallback, this, ros::TransportHints().tcpNoDelay());
        current_point_sub = nh.subscribe<geometry_msgs::Point>("/current_point", 10, &Process_Control::current_pointCallback, this, ros::TransportHints().tcpNoDelay());
        tof_distance_sub = nh.subscribe<std_msgs::Float64>("/tof_distance", 10, &Process_Control::tof_distance_callback, this, ros::TransportHints().tcpNoDelay());
        x_real_error_sub = nh.subscribe<std_msgs::Float64>("/x_real_error", 10,&Process_Control::x_real_error_callback, this, ros::TransportHints().tcpNoDelay());
        side_pixel_error_sub = nh.subscribe<std_msgs::Float64>("/side_pixel_error", 10,&Process_Control::side_pixel_error_callback, this, ros::TransportHints().tcpNoDelay());
        odometrySubscriber = nh.subscribe("/odom_gps", 10, &Process_Control::odometryCallback, this, ros::TransportHints().tcpNoDelay());//订阅gps里程   
        numLabelSubscriber = nh.subscribe<std_msgs::Int32>("/num_label", 10, &Process_Control::numlabelCallback, this, ros::TransportHints().tcpNoDelay());
        markCompletedSub = nh.subscribe<std_msgs::Int32>("/markcompleted", 10, &Process_Control::markCompletedCallback, this, ros::TransportHints().tcpNoDelay());

        startTrackingSub = nh.subscribe<std_msgs::Bool>("/starttracking", 10, &Process_Control::startTrackingCallback, this, ros::TransportHints().tcpNoDelay());
        startLineFollowSub = nh.subscribe<std_msgs::Bool>("/startlinefollow", 10, &Process_Control::startLineFollowCallback, this, ros::TransportHints().tcpNoDelay());
        startScrapSub = nh.subscribe<std_msgs::Bool>("/startscrap", 10, &Process_Control::startScrapSubCallback, this, ros::TransportHints().tcpNoDelay());
        startInitSub = nh.subscribe<std_msgs::Bool>("/startinit", 10, &Process_Control::startInitCallback, this, ros::TransportHints().tcpNoDelay());
        startTaskSub = nh.subscribe<std_msgs::Bool>("/starttask", 10, &Process_Control::startTaskCallback, this, ros::TransportHints().tcpNoDelay());
        trackingCompletedSub = nh.subscribe<std_msgs::Bool>("/trackingCompleted", 10, &Process_Control::trackingCompletedCallback, this, ros::TransportHints().tcpNoDelay());
        imuOdomSub = nh.subscribe<nav_msgs::Odometry>("/odom", 10, &Process_Control::imuOdomCallback, this, ros::TransportHints().tcpNoDelay());//订阅IMU里程计
        ultraSonicSub = nh.subscribe<sensor_msgs::Range>("/ultrasonic_distance", 10, &Process_Control::ultrasonicCallBack, this, ros::TransportHints().tcpNoDelay());//
        navFixSub = nh.subscribe<sensor_msgs::NavSatFix>("/fix", 10, &Process_Control::navFixCallBack, this, ros::TransportHints().tcpNoDelay());//
        _getTofDistance = nh.serviceClient<robot_bringup::get_tof_distance>("/get_tof_distance");
        

        state_buf.data = true;
        LR_control_enable_pub.publish(state_buf);// 打开PID控制
    }
    ~Process_Control(){};

    /*保留函数，供双目测距使用*/
    geometry_msgs::Point cal_point_by_vision(int _a, int _b)
    {
        geometry_msgs::Point point_in_scrap;
        return point_in_scrap;
    }

    void setThreshold(float threshold)
    {
        this->ultrasonicDistanceThreshold = threshold;
    }

    int processStart(std::vector<allPkgLaunchList> &startList)
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
                    std::vector<allPkgLaunchList>::iterator iter = std::find(pkgLaunchAllList.begin(), pkgLaunchAllList.end(), startList[i]);
                    if(iter == pkgLaunchAllList.end())  //当前列表中没有此进程
                    {
                        std::cout << "["<< startList[i].pkgname << "]:pid is " << '[' << getpid() << ']' << std::endl; 
                        execlp("roslaunch", "roslaunch", startList[i].pkgname.c_str(), startList[i].launchname.c_str(), NULL);
                        exit(0); 
                    }
                    else
                    {
                        ROS_ERROR("%s %s is allready running.", iter->pkgname.c_str(), iter->launchname.c_str());
                        exit(-1); 
                    }               
                }
            }
        }

        std::cout   << "------------------this is main process--------------------" << std::endl \
                    << pkgLaunchAllList.at(0).pkgname << " pid:" << pkgLaunchAllList[0].pid << std::endl \
                    << pkgLaunchAllList.at(1).pkgname << " pid:" << pkgLaunchAllList[1].pid << std::endl ;

        return 0;
    }

    int processStop(std::vector<allPkgLaunchList> &stopList)
    {
        int ret, status, pid;

        for(int i = 0; i< stopList.size(); i++)
        {
            std::vector<allPkgLaunchList>::iterator iter = std::find(pkgLaunchAllList.begin(), pkgLaunchAllList.end(), stopList[i]);
            if(iter != pkgLaunchAllList.end())//查找成功
            {
                ret = kill(iter->pid, SIGINT);
                pid = wait(&status);
                if(0 == ret)
                {
                    ROS_INFO("%s %s is killed successfully. PID[%d]", iter->pkgname.c_str(), iter->launchname.c_str(), pid);
                    pkgLaunchAllList.erase(iter);   
                }
                else
                {
                    ROS_ERROR("%s %s killing failed.", iter->pkgname.c_str(), iter->launchname.c_str());
                    return -1;
                }
            }
            else
            {
                ROS_ERROR("do not found %s %s process in global list", stopList[i].pkgname.c_str(), stopList[i].launchname.c_str());
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

    void side_pixel_error_callback(const std_msgs::Float64::ConstPtr &msg)
    {
        _side_pixel_error = msg->data;
    }

    void current_pointCallback(const geometry_msgs::PointConstPtr &point_msg)
    {
        current_point.x = point_msg->x;
        current_point.z = point_msg->z;
    }

    void startInitCallback(const std_msgs::Bool::ConstPtr &msg)
    {
        std::vector<allPkgLaunchList> processList = 
        {
            {"robot_bringup", "bringup.launch", 0},
            {"arm_move", "arm_move.launch", 0},
            {"line_follow", "line_follow_pid.launch", 0},
            {"geo_to_enu", "geo_to_enu.launch", 0},
            {"pure_pursuit_controller", "map_odom.launch", 0},
            {"pure_pursuit_controller", "pure_pursuit_odom.launch", 0}
        };//
        if(msg->data == true)
        {
            this->processStart(processList);
            processList.clear();
            sleep(2);
            processList =
            {
                {"pure_pursuit_controller", "pure_pursuit.launch", 0},
                {"pure_pursuit_controller", "test_send_waypoints.launch", 0}
            };
            this->processStart(processList);
        }
        else
        {
            processList =
            {
                {"robot_bringup", "bringup.launch", 0},
                {"arm_move", "arm_move.launch", 0},
                {"line_follow", "line_follow_pid.launch", 0},
                {"geo_to_enu", "geo_to_enu.launch", 0},
                {"pure_pursuit_controller", "map_odom.launch", 0},
                {"pure_pursuit_controller", "pure_pursuit_odom.launch", 0},
                {"pure_pursuit_controller", "pure_pursuit.launch", 0},
                {"pure_pursuit_controller", "test_send_waypoints.launch", 0}
            };
            this->processStop(processList);
        }
        sendFlagBool(side_camera_run_pub, true);  //开启侧视相机识
        _initFlag = true;
    }

    void startTrackingCallback(const std_msgs::Bool::ConstPtr &msg)
    {
        std::vector<allPkgLaunchList> processList = 
        {
            {"geo_to_enu", "geo_to_enu.launch", 0},
            {"pure_pursuit_controller", "pure_pursuit.launch", 0}
        };
        if(msg->data == true)
        {
            this->processStart(processList);
            processList.clear();
            sleep(2);
            processList = 
            {
                {"pure_pursuit_controller", "test_send_waypoints.launch", 0}
            };  
            this->processStart(processList);          
        }
        else
        {
            processList = 
            {
                {"geo_to_enu", "geo_to_enu.launch", 0},
                {"pure_pursuit_controller", "pure_pursuit.launch", 0},
                {"pure_pursuit_controller", "pure_pursuit_odom.launch", 0},
                {"pure_pursuit_controller", "test_send_waypoints.launch", 0}
            };
            this->processStop(processList);
        }
    }

    void startLineFollowCallback(const std_msgs::Bool::ConstPtr &msg)
    {
        std::vector<allPkgLaunchList> processList = 
        {
            {"line_follow", "line_follow_pid.launch", 0}
        }; 
        if(msg->data == true)
        {
            //system("python yolox-tf2-main-loc_vertical_view/loc_predict.py");
            this->processStart(processList);
        }
        else
        {
            this->processStop(processList);
        }       
    }

    void startScrapSubCallback(const std_msgs::Bool::ConstPtr &msg)
    {
        std::vector<allPkgLaunchList> processList = 
        {
            {"arm_move", "differential_drive_sim.launch", 0},
        };
        if(msg->data == true)
        {
            this->processStart(processList);
        }
        else
        {
            this->processStop(processList);
        }
    }

    void trackingCompletedCallback(const std_msgs::Bool::ConstPtr &msg)
    {
        _trackingCompletedFlag = msg->data;
        ROS_INFO_STREAM_ONCE("get msg from pure_pursuit node");
    }

    void numlabelCallback(const std_msgs::Int32::ConstPtr &msg)
    {
        _num_label = msg->data;
    }

    void markCompletedCallback(const std_msgs::Int32::ConstPtr &msg)
    {
        mark_state = msg->data;
    }

    void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg) 
    {	
        _currentPose.header = msg->header;
        _currentPose.pose = msg->pose.pose;
    }

    void ultrasonicCallBack(const sensor_msgs::Range::ConstPtr& msg) 
    {
        ultrasonicDistance = msg->range;
    }

    void imuOdomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        tf::Quaternion q; 
        tf::quaternionMsgToTF(msg->pose.pose.orientation, q);
        tf::Matrix3x3(q).getRPY(roll,pitch,yaw);
    }

    bool getTaskState(void)
    {
        return this->_taskFlag;
    }

    void navFixCallBack(const sensor_msgs::NavSatFix::ConstPtr& msg)
    {
        if(msg->status.status == msg->status.STATUS_SBAS_FIX)
        {
            rtk_aviliable = true;
        }
        else
        {
            rtk_aviliable = false;
        }
    }

    void sendFlagBool(ros::Publisher pub, bool flag)
    {
        std_msgs::Bool msg;
        msg.data = flag;
        pub.publish(msg);
    }

    void sendFlagInt32(ros::Publisher pub, int32_t mydata)
    {
        std_msgs::Int32 pubmsg;
        pubmsg.data = mydata;
        pub.publish(pubmsg);
    }

    // template <typename T>
    // void sendMsgToNode(ros::Publisher pub, T msg_data)
    // {
    //     std_msgs::Bool msg;
    //     msg.data = flag;
    //     pub.publish(msg);
    // }

    void startTaskCallback(const std_msgs::Bool::ConstPtr &msg)
    {
        _taskFlag = msg->data;
        if(!_taskFlag)
        {
            sendFlagBool(line_run_pub, false);
            sendFlagBool(rtk_run_pub, false);
        }
    }

    int  scrap_process(void)
    {
        std_msgs::Float64 big_deg, small_deg;
                        std::string ret;
                        std::cout << "waiting cin to zhuaqu " << std::endl;
                        std::cin >> ret;

        int_buf.data = 79500;
        abs_pub.publish(int_buf);//横向运行到中间
        // while (!arm_is_ok);//等待上次占用结束
        // point_buf.x = 0.59; // x_distance;
        if(x_distance > 68)
        {
            ROS_ERROR("[process_control] x_distance is exceeded or Al is not be seen!");
        }
        // point_buf.x = (x_distance - 10) / 100;
        point_buf.x = 0.55;
        if(ret == "1")
        {
            point_buf.z = -0.15;
        }
        else if(ret == "2")
        {
            point_buf.z = -0.12;
        }
        else if(ret == "3")
        {
                point_buf.z = -0.11;
        }
        //point_buf.z = -0.14;
        target_point_pub.publish(point_buf);//初步对准x，并下放夹爪
        ROS_INFO("culue down x=%f z=%f", point_buf.x, point_buf.z);
        // sleep(17);
                        std::cout << "waiting cin to jingzhun" << std::endl;
                        std::cin >> ret;

        /*精准下放夹爪到15cm高度*/
        // getTofDistance();
        // ROS_INFO("z_distance = %f", tof_distance);
        //             std::cout << "waiting cin to down 15cm(bs)" << std::endl;
        //             std::cin >> ret;
        // if(std::abs(tof_distance) > 10.0)
        // {
        //     point_buf.z = current_point.z - tof_distance/1000;
        //     target_point_pub.publish(point_buf);//下放夹爪
        //     // sleep(2);
        //     // while (!arm_is_ok);//等待上次占用结束
        // }      

        ROS_INFO("jingzhun15cm down x=%f z=%f", point_buf.x, point_buf.z);

                        std::cout << "waiting cin to duizhong" << std::endl;
                        std::cin >> ret;

        /*进入PID调中控制*/
        data_buf.data = 0.0;
        LR_target_pub.publish(data_buf);
        state_buf.data = true;
        LR_control_enable_pub.publish(state_buf);// 打开PID控制
        /*测量z轴高度差*/

                        std::cout << "waiting cin to test z distance" << std::endl;
                        std::cin >> ret;
        // sleep(8);
        state_buf.data = false;
        LR_control_enable_pub.publish(state_buf);// guanPID控制
        // sleep(1);
        ROS_INFO("X_ERROR = %f", x_real_error/100);

        point_buf.x = current_point.x + x_real_error/100;
        point_buf.z = current_point.z - 0.10;
        ROS_INFO("last x = %f  |  z = %f", point_buf.x, point_buf.z);
                        std::cout << "waiting cin to down 10cm(realative)" << std::endl;
                        std::cin >> ret;
        target_point_pub.publish(point_buf);//下放夹爪
        // sleep(2);
        // while (!arm_is_ok);//等待上次占用结束
                

                        std::cout << "waiting cin to scrap" << std::endl;
                        std::cin >> ret;
        /*抓取*/
        arm_buf.stepper_two = -1;//负数为抓
        arm_buf.almo_one = 0;
        arm_buf.almo_two = 0;
        arm_buf.stepper_one = 0;
        arm_motion_pub.publish(arm_buf);
        // sleep(6);

                        std::cout << "waiting cin to return" << std::endl;
                        std::cin >> ret;
        point_buf.x = 0.45;
        point_buf.z = -0.05;
        ROS_INFO("x = %f  |  z = %f", point_buf.x, point_buf.z);
        target_point_pub.publish(point_buf);//下放夹爪

        int_buf.data = 0;
        abs_pub.publish(int_buf);//横向运行到初始位

        // sleep(12);

                        // std::cout << "waiting cin to loosen" << std::endl;
                        // std::cin >> ret;

        // arm_buf.stepper_two = 1;//正数为释放
        // arm_buf.almo_one = 0;
        // arm_buf.almo_two = 0;
        // arm_buf.stepper_one = 0;
        // arm_motion_pub.publish(arm_buf);

                        std::cout << "waiting cin to go to mark position" << std::endl;
                        std::cin >> ret;

        int_buf.data = -78500;
        abs_pub.publish(int_buf);//横向运行到初始位

        point_buf.x = 1000.0;
        point_buf.z = 1000.0;//这个值无解 不起作用
        target_point_pub.publish(point_buf);//发送出去用于停止判断

        big_deg.data = -2.1973;//转到打标位置
        small_deg.data = -16.875;//转到打标位置
        big_target_joint_deg_pub.publish(big_deg);
        small_target_joint_deg_pub.publish(small_deg);

        std_msgs::Bool buf;
        buf.data = true;
        pid_enable_pub.publish(buf);

                        std::cout << "waiting cin to reach mark position" << std::endl;
                        std::cin >> ret;
        // sleep(12);
        // buf.data = false;
        // pid_enable_pub.publish(buf);     
        return 0;
    }

    void robotComeBack(void)
    {
        /*--------------------------------------原地掉头-----------------------------------------------*/
        log_to_pc("------Come Back waiting manual caozuo------");  
        std::string ret;
        std::cout << "waiting cin to diaotou" << std::endl;
        std::cin >> ret;
        // /*重启bring里程计/
        // std::vector<allPkgLaunchList> processList = 
        // {
        //     {"robot_bringup", "bringup.launch", 0}//需重启里程计
        // };  
        // this->processStop(processList);
        // sleep(2); 
        // this->processStart(processList);
        // geometry_msgs::Twist cmdvel;
        // while(1)
        // {
        //     cmdvel.angular.z = 0.2;
        //     cmd_vel_pub.publish(cmdvel);
        //     usleep(50000); 
        // }
        /*--------------------------------------开始巡线----------------------------------------------*/
        log_to_pc("------Start Line Follow------");
        std_msgs::Int32 buf;
        buf.data = 300;
        pid_targrt_vel_pub.publish(buf);
        sleep(2);
        sendFlagBool(line_run_pub, true);
        std::cout << "waiting cin to reached final" << std::endl;  
        std::cin >> ret;
        sendFlagBool(line_run_pub, false);  
        /*----------------------------------到达黄线终点，开始返回--------------------------------------*/
        log_to_pc("------Turn Right 90------");
        /*更改为发布路径圆*/
        std_msgs::Int32 databuf;
        // databuf.data = 4;
        // track_path_switch_pub.publish(databuf);
        // usleep(500000);  
        // std::vector<allPkgLaunchList> processList = 
        // {
        //     {"robot_bringup", "bringup.launch", 0},//需重启里程计
        //     {"line_follow", "line_follow_pid.launch", 0}//关闭黄线
        // };        
        // this->processStop(processList);
        // sleep(2);
        // processList.clear();
        // processList = 
        // {
        //     {"robot_bringup", "bringup.launch", 0},
        //     {"pure_pursuit_controller", "pure_pursuit_odom.launch", 0}
        // };  
        // this->processStart(processList);
        // sleep(2);
        // processList.clear();
        // processList = 
        // {
        //     {"pure_pursuit_controller", "test_send_waypoints.launch", 0}
        // };   
        // this->processStart(processList);
        // sleep(1);
        // /*启动odom循迹*/
        // sendFlagBool(rtk_run_pub,true);
        // log_to_pc("------odom_Tracking_Started------");
        // while(!_trackingCompletedFlag)
        // {
        //     sleep(1);
        //     log_to_pc("------odom_Tracking------");
        // }
        // _trackingCompletedFlag = false; //到达odom终点
        // log_to_pc("------Tracking Reached------");


        std::cout << "waiting cin to restart RTK" << std::endl;  
        std::cin >> ret; //-------------------------------------------------------------------------------------------------------------
        /*-----------------------开启RTK导航---------------------------*/
        log_to_pc("------Start RTK Back------");
        // processList.clear();
        std::vector<allPkgLaunchList> processList = 
        {
            {"geo_to_enu", "geo_to_enu.launch", 0},
            {"pure_pursuit_controller", "test_send_waypoints.launch", 0}
        };
        this->processStart(processList);
        std::cout << "waiting cin to if restart RTK" << std::endl;  
        std::cin >> ret;
        if(ret == "restart")
        {
            this->processStop(processList);       
            sleep(3);
            this->processStart(processList);       
        }        
        sleep(2);
        std::cout << "waiting cin to waiting RTK" << std::endl;  
        std::cin >> ret;
        /*更换为rtk返回路径*/
        databuf.data = 3;
        track_path_switch_pub.publish(databuf);
        sleep(2);
        processList = 
        {
            {"pure_pursuit_controller", "pure_pursuit.launch", 0}
        };        
        this->processStart(processList);
        sleep(4);
        sendFlagBool(rtk_run_pub,true); 
        while(!_trackingCompletedFlag)
        {
            sleep(1);
            log_to_pc("------rtk_Back_Tracking------");
        }
        _trackingCompletedFlag = false; //到达RTK终点
        log_to_pc("------Tracking Reached------");   
        std::cout << "waiting cin to replay" << std::endl;  
        std::cin >> ret; //---            
    }

    void line_follow_process(void)
    {
        log_to_pc("--------- Line Follow & Scrap ---------");
        std::string ret;
                    std::cout << "waiting cin to line follow start" << std::endl;
                    std::cin >> ret;
        /*开始巡线*/
        std_msgs::Int32 buf;
        buf.data = 200;
        pid_targrt_vel_pub.publish(buf);
        sleep(2);
        sendFlagBool(line_run_pub, true);
        {  
       // std::cout << "waiting cin to Stop" << std::endl;
       // std::cin >> ret;
        for(int i = 0;i < 3; i++)
        {
            log_to_pc("--------- Start Line Tracking ---------"); //
            ROS_INFO("--------- Start Line Tracking ---------");
            /*若找到铝锭，并到达中心，则停止，否则一直走*/
            while(_side_pixel_error < -700)
            {
                if(this->_taskFlag == false)
                {
                    return;
                }
                usleep(10000);                    
            };
            ROS_INFO("--------- looked ---------");
            buf.data = 100;
            pid_targrt_vel_pub.publish(buf);
            while(std::abs(_side_pixel_error) > 200.0f)
            {
                if(this->_taskFlag == false)                
                {
                    return;
                }  
                usleep(10000);  
            }
            ROS_INFO("--------- Found Al mid point ---------");
            log_to_pc("--------- Found Al mid point ---------");
            sendFlagBool(line_run_pub, false);
            sleep(2);
            if(_side_pixel_error > 200)
            {
                weitiao();
            } 
         //   scrap_process();
            std::cout << "get num label: " << _num_label << std::endl;
            /*读取标牌并打标*/
            sleep(3);//等待3s的延时
            mark_state = 0;
            std_msgs::Int16 databuf;
            databuf.data = _num_label;//

                    std::cout << "waiting cin" << std::endl;
                    std::cin >> ret;
                    databuf.data = atoi(ret.c_str());
                    printf("set databuf.data: %d", databuf.data);
            //std::cout << "set databuf.data: " << databuf.data << std::endl;
            mark_pub.publish(databuf);
            /*等待打标返回*/
            while(!mark_state)
            {
                if(this->_taskFlag == false)                
                {
                    return;
                }  
                usleep(10000); 
            }
            if(mark_state == 1)
            {
                log_to_pc("--------- mark Completed ---------"); 
            }
            else if(mark_state == -1)
            {
                log_to_pc("--------- mark Failed ---------"); 
            }
            arm_buf.stepper_two = 1;//正数为释放
            arm_buf.almo_one = 0;
            arm_buf.almo_two = 0;
            arm_buf.stepper_one = 0;
            arm_motion_pub.publish(arm_buf);
            buf.data = 200;
            pid_targrt_vel_pub.publish(buf);
            sleep(6);
            sendFlagBool(line_run_pub, true);
        }
        }
        sendFlagBool(line_run_pub, false);
        log_to_pc("--------- line follow completed ---------"); 
        std::cout << "waiting comeback" << std::endl;
        std::cin >> ret; 
    }

    void weitiao(void)
    {
        geometry_msgs::Twist cmdVelocity;
        for(int i = 0; i < 10*3 ; i++)
        {
            cmdVelocity.linear.x = -0.05;
            cmd_vel_pub.publish(cmdVelocity);
            usleep(100000);
        }
        cmdVelocity.linear.x = 0.0;
        cmd_vel_pub.publish(cmdVelocity);     
    }

    void all_process(void)
    {
        if(this->_taskFlag)
        {
            sendFlagBool(odom_run_pub, true);
            log_to_pc("------odom_Tracking_Started------");
            while (!_trackingCompletedFlag)
            {
                sleep(1);
                log_to_pc("------odom_Tracking------");
            }
            _trackingCompletedFlag = false; //到达odom终点
            sendFlagBool(odom_run_pub, false);

            /*巡线and抓取*/
            sendFlagInt32(pid_targrt_vel_pub, 200);
            sleep(1);
            sendFlagBool(line_run_pub, true);
            sleep(20);
            sendFlagBool(line_run_pub, false);

            while (!rtk_aviliable)//等待rtk信号变成E5
            {
                sleep(1);
            }
            this->_taskFlag = false;//关闭任务
        }
    }
    
    // void all_process(void)
    // {
    //     if (this->_taskFlag)
    //     {
    //         sendFlagBool(rtk_run_pub, true);
    //         log_to_pc("------rtk_Tracking_Started------");
    //         sendFlagBool(side_camera_run_pub, true);//直接打开侧视识别程序
    //         while (!_trackingCompletedFlag)
    //         {
    //             sleep(1);
    //             log_to_pc("------rtk_Tracking------");
    //         }
    //         _trackingCompletedFlag = false; //到达RTK终点
    //         log_to_pc("------Tracking Reached------");

    //         /*关闭RTK循迹*/
    //         std::vector<allPkgLaunchList> processList =
    //         {
    //             {"geo_to_enu", "geo_to_enu.launch", 0},
    //             {"pure_pursuit_controller", "pure_pursuit.launch", 0},
    //             {"robot_bringup", "bringup.launch", 0}//需重启里程          
    //         };
    //         this->processStop(processList);
    //         /*关闭路径发布*/
    //         std_msgs::Int32 databuf;
    //         databuf.data = 0;
    //         track_path_switch_pub.publish(databuf);

    //         /*开启TF静态发布线程*/
    //         tf_flag = 0;
    //         staticTFBroadcasterThread = new std::thread(&Process_Control::tfBroadcaster, this);
    //         staticTFBroadcasterThread->detach();
    //         sleep(2);
    //         processList.clear();
    //         processList =
    //         {
    //             {"robot_bringup", "bringup.launch", 0},//需重启里程计
    //             {"pure_pursuit_controller", "pure_pursuit_odom.launch", 0}
    //         };
    //         this->processStart(processList);
    //         sleep(2);
    //         processList =
    //         {
    //             {"pure_pursuit_controller", "test_send_waypoints.launch", 0}
    //         };
    //         this->processStart(processList);
    //         sleep(1);
    //         /*更改为发布路径圆*/
    //         databuf.data = 2;
    //         track_path_switch_pub.publish(databuf);
    //         usleep(500000);
    //         /*启动odom循迹*/
    //         sendFlagBool(rtk_run_pub, true);
    //         log_to_pc("------odom_Tracking_Started------");
    //         while (!_trackingCompletedFlag)
    //         {
    //             sleep(1);
    //             log_to_pc("------odom_Tracking------");
    //         }
    //         _trackingCompletedFlag = false; //到达odom终点
    //         log_to_pc("------Tracking Reached------");
    //         processList =
    //         {
    //             {"pure_pursuit_controller", "pure_pursuit_odom.launch", 0}
    //             //{"pure_pursuit_controller", "test_send_waypoints.launch", 0}
    //         };
    //         sleep(1);
    //         this->processStop(processList);
    //         tf_flag = 0;
    //         // pthread_cancel(staticTFBroadcasterThread->native_handle());//关闭静态TF发布
    //         processList =
    //         {
    //             {"line_follow", "line_follow_pid.launch", 0}
    //         };
    //         this->processStart(processList);
    //         std::string ret;
    //         std::cout << "waiting cin to line follow" << std::endl;
    //         std::cin >> ret;
    //         /*开始寻黄线 抓取流程*/
    //         line_follow_process();
    //         /*执行返回流程*/
    //         sleep(3);
    //         robotComeBack();
    //         this->_taskFlag = false;//关闭任务
    //     }
    // }
    
    void sendStaticTransform(void)
    {
        // 创建 tf2 要广播的静态坐标变换        
        geometry_msgs::TransformStamped static_transform_stamped;
        if(tf_flag == 0)
        {
            // 对坐标变换初始化
            static_transform_stamped.header.stamp = ros::Time::now();
            static_transform_stamped.header.frame_id = "map";
            static_transform_stamped.child_frame_id = "odom";
            // 初始化x y z
            static_transform_stamped.transform.translation.x = _currentPose.pose.position.x;
            static_transform_stamped.transform.translation.y = _currentPose.pose.position.y;
            static_transform_stamped.transform.translation.z = _currentPose.pose.position.z;
            // 初始化四元数
            static_transform_stamped.transform.rotation.x = _currentPose.pose.orientation.x;
            static_transform_stamped.transform.rotation.y = _currentPose.pose.orientation.y;
            static_transform_stamped.transform.rotation.z = _currentPose.pose.orientation.z;
            static_transform_stamped.transform.rotation.w = _currentPose.pose.orientation.w;
            // tf2 广播对象发布静态坐标变换
            static_broadcaster.sendTransform(static_transform_stamped);
        }
        else
        {
            // 对坐标变换初始化
            static_transform_stamped.header.stamp = ros::Time::now();
            static_transform_stamped.header.frame_id = "map";
            static_transform_stamped.child_frame_id = "odom";
            // 初始化x y z
            static_transform_stamped.transform.translation.x = 0;
            static_transform_stamped.transform.translation.y = 0;
            static_transform_stamped.transform.translation.z = 0;
            // 初始化四元数
            static_transform_stamped.transform.rotation.x = 0;
            static_transform_stamped.transform.rotation.y = 0;
            static_transform_stamped.transform.rotation.z = 0;
            static_transform_stamped.transform.rotation.w = 1;
            // tf2 广播对象发布静态坐标变换
            static_broadcaster.sendTransform(static_transform_stamped);            
        }
        // ROS_INFO("Sent TF from map to odom!");
        // log_to_pc("Sent TF from map to odom!");
    }

    void tfBroadcaster(void)
    {
        ros::Rate r(10);
        while (ros::ok())
        {   
            sendStaticTransform();
            r.sleep();
        }
    }

    void getTofDistance(void)
    {
        robot_bringup::get_tof_distance srv;
        bool ret = false;
        do{
            ret = _getTofDistance.call(srv);
            if(ret)
            {
                tof_distance = srv.response.distance;
                return;
            }
            sleep(2);
            ROS_ERROR("called get distance srv default! retrying");
        }while(ret == false);
    }

    void chargeProcess(void)
    {
        int i = 0;
        std_msgs::Int32 buf;
        buf.data = 100;
        pid_targrt_vel_pub.publish(buf);
        sleep(2);
        sendFlagBool(line_run_pub, true); //此时开始巡线了
        /*不断判断是否达到充电位置*/
        while ( ultrasonicDistance > ultrasonicDistanceThreshold)
        {
            if (!this->_taskFlag)
            {
                break;
            }
            usleep(1000);
            i++;
            if(i > 1000)
            {
                i = 0;
                std::cout << "not arrive charge point ultrasonic = [" << ultrasonicDistance \
                    << "] " << ultrasonicDistanceThreshold << std::endl;
            }
        }
        /*停止条件*/
        ROS_INFO("STOP and Start Charging");
        sendFlagBool(line_run_pub, false);
    }
};


void rosSpin(void)
{
    ros::spin();
}


void siginthandler(int sig)
{
    ROS_INFO("shutting down!");
    ros::shutdown();
    exit(0);
}


/* main */
int main(int argc, char* argv[])
{
    int status,ret;

    ros::init(argc, argv, "process_control");

    if (argc != 2)
    {
        ROS_ERROR("usage: add_ONE_ints_client ultrasonicThreshold");
        return 1;
    }

    std::cout << "ultrasonicThreshold = " << atof(argv[1]) << std::endl;
    signal(SIGINT, siginthandler);
    Process_Control PC;

    /*设置超声波停止阈值*/
    PC.setThreshold(atof(argv[1]));

    std::thread rosSpinThread(rosSpin);

    rosSpinThread.detach();
    ros::Rate r(1);
    int i = 0;

    while (ros::ok())
    {   
        // PC.chargeProcess();
        // PC.all_process();
        // PC.line_follow_process();
        // PC.robotComeBack();
        PC.scrap_process();
        // i = getchar();
        // PC.robotComeBack();
        // std::string ret;
        // std::cout << "waiting cin" << std::endl;
        // std::cin >> ret;
        // int16_t set = atoi(ret.c_str());
        // printf("set databuf.data: %d", set);
        PC.log_to_pc("------i am alive------");
        r.sleep();
    }
    
    return 0;
}
