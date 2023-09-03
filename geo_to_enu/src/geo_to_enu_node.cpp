/*
 * @Author: Wei Zhifei
 * @Date: 2023-03-17 19:13:53
 */
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>

#include <GeographicLib/LocalCartesian.hpp> //包含头文件

#include <iostream>
#include <fstream>
#include <iomanip>

using namespace std;


GeographicLib::LocalCartesian geo_converter;

// ROS订阅者和发布者
ros::Subscriber gps_data_sub;
ros::Subscriber heading_sub;
ros::Publisher path_pub;
ros::Publisher odom_pub;
// 全局变量
string gps_sub_topic = "";
string output_frame_name ="";
double z_rotate_value = 0.0;
bool init_flag = true;
bool auto_init;
bool write_flag;
nav_msgs::Path path;
nav_msgs::Odometry odom;
double latitude_origin,longitude_origin;
fstream fs;
fstream fs2;
fstream fs3;

void heading_callback(const geometry_msgs::QuaternionStamped::ConstPtr& heading_msg)
{
    // tf::Vector3 v1(0,0,1);
    // tf::Quaternion q1; //x,y,z,w
    // q1.setW(heading_msg->quaternion.w);
    // q1.setX(heading_msg->quaternion.x);
    // q1.setY(heading_msg->quaternion.y);
    // q1.setZ(heading_msg->quaternion.z);
    // q1.setRotation(v1, M_PI);
    // odom.pose.pose.orientation.w = q1.getW();
    // odom.pose.pose.orientation.x = q1.getX();
    // odom.pose.pose.orientation.y = q1.getY();
    // odom.pose.pose.orientation.z = q1.getZ();//
    odom.pose.pose.orientation.w = heading_msg->quaternion.w;
    odom.pose.pose.orientation.x = heading_msg->quaternion.x;
    odom.pose.pose.orientation.y = heading_msg->quaternion.y;
    odom.pose.pose.orientation.z = heading_msg->quaternion.z;//
}
void gps_to_xyz(const sensor_msgs::NavSatFix::ConstPtr& gps_msg)
{
	double local_E, local_N, local_U;
    if (init_flag == true)
    {
        if(auto_init==true)
        {
            fs2 << "W: " ;
            fs2 << fixed << setprecision(9) <<gps_msg->latitude << endl ;
            fs2 << "E: " ;
            fs2 << fixed << setprecision(9)<< gps_msg->longitude;
            geo_converter.Reset(gps_msg->latitude, gps_msg->longitude, gps_msg->altitude);
        }
        else
        {
            geo_converter.Reset(latitude_origin, longitude_origin, gps_msg->altitude);
        }
        init_flag = false;
    }
    else
    {
	geo_converter.Forward(gps_msg->latitude, gps_msg->longitude, gps_msg->altitude, local_E, local_N, local_U);
        if (path_pub.getNumSubscribers() > 0)
        {
            geometry_msgs::PoseStamped this_pose_stamped;
            this_pose_stamped.header.frame_id = output_frame_name;
            this_pose_stamped.pose.position.x = local_E;
            this_pose_stamped.pose.position.y = local_N;
            this_pose_stamped.pose.position.z = 0;
            this_pose_stamped.pose.orientation.x = odom.pose.pose.orientation.x;
            this_pose_stamped.pose.orientation.y = odom.pose.pose.orientation.y;
            this_pose_stamped.pose.orientation.z = odom.pose.pose.orientation.z;
            this_pose_stamped.pose.orientation.w = odom.pose.pose.orientation.w;
            path.poses.push_back(this_pose_stamped);
            path.header.frame_id = output_frame_name;
        }
        path_pub.publish(path);

        if (odom_pub.getNumSubscribers() > 0)
        {
            odom.header.frame_id = output_frame_name;
            odom.header.stamp = ros::Time::now();
            odom.pose.pose.position.x = local_E;
            odom.pose.pose.position.y = local_N;
            odom.pose.pose.position.z = 0;
        }
        odom_pub.publish(odom);
        if(write_flag)
        {
            fs << local_E <<" "<< local_N <<" " << odom.pose.pose.orientation.x <<" "
                                                << odom.pose.pose.orientation.y <<" "
                                                << odom.pose.pose.orientation.z <<" "
                                                << odom.pose.pose.orientation.w << endl;
            fs3 << local_E <<" "<< local_N <<" " << odom.pose.pose.orientation.x <<" "
                                                << odom.pose.pose.orientation.y <<" "
                                                << odom.pose.pose.orientation.z <<" "
                                                << odom.pose.pose.orientation.w << endl;                                    
        }
    }
}
int main(int argc,char **argv)
{
    ros::init(argc,argv,"geo_transto_enu");
    ros::NodeHandle nh;
    nh.param<string>("gps_sub_topic", gps_sub_topic, "/gps");
    nh.param<string>("output_frame_name", output_frame_name, "map");
    nh.param<bool>("auto_get_origin_gps", auto_init, true);
    nh.param<double>("z_rotate_value", z_rotate_value, 1.0);
    nh.param<bool>("write_flag", write_flag, false);

    nh.param<double>("lat_origin", latitude_origin, 119.665344097);
    nh.param<double>("lon_origin", longitude_origin, 45.4516301975);

    gps_data_sub = nh.subscribe<sensor_msgs::NavSatFix>(gps_sub_topic, 1000, gps_to_xyz);
    heading_sub = nh.subscribe<geometry_msgs::QuaternionStamped>("/heading", 1000, heading_callback);
	path_pub = nh.advertise<nav_msgs::Path>("/gpsTrack",1, true);
    odom_pub = nh.advertise<nav_msgs::Odometry>("/odom_gps",1,true);

    if(auto_init == true)
    {
        system("cd /home/nvidia/ && mkdir rtk_path");
        system("cd /home/nvidia/rtk_path && touch path.txt");
        system("cd /home/nvidia/rtk_path && touch origin.txt");
        system("cd /home/nvidia/rtk_path && touch last_pose.txt");
        fs3.open("/home/nvidia/rtk_path/last_pose.txt",ios::out);
        fs2.open("/home/nvidia/rtk_path/origin.txt",ios::out);
        fs.open("/home/nvidia/rtk_path/path.txt",ios::out);
    }
        
    ros::spin();
    fs.close();
    fs2.close();
    fs3.close();
    return 0;
}
