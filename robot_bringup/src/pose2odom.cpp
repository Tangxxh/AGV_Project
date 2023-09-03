//
// Created by smith on 7/24/20.
//

#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

class OdomEKFTransform
{
    public:
        OdomEKFTransform()
        {
            pub_ = n_.advertise<nav_msgs::Odometry>("/odom", 100);
            sub_ = n_.subscribe("/robot_pose_ekf/odom_combined", 100, &OdomEKFTransform::callback, this, ros::TransportHints().tcpNoDelay());
        }

        void callback(const geometry_msgs::PoseWithCovarianceStamped& msg)
        {
            nav_msgs::Odometry odom;
            geometry_msgs::TransformStamped odom_trans;
            static tf::TransformBroadcaster odom_broadcaster;

            odom.header = msg.header;
            odom.header.frame_id = "odom";
            odom.pose = msg.pose;
            pub_.publish(odom);

            odom_trans.header.stamp = ros::Time::now();
            odom_trans.header.frame_id = "odom";
            odom_trans.child_frame_id = "base_footprint";
            odom_trans.transform.translation.x = odom.pose.pose.position.x;
            odom_trans.transform.translation.y = odom.pose.pose.position.x;
            odom_trans.transform.translation.z = 0.0;
            odom_trans.transform.rotation = odom.pose.pose.orientation;
            odom_broadcaster.sendTransform(odom_trans);
        }

    private:
        ros::NodeHandle n_;
        ros::Publisher pub_;
        ros::Subscriber sub_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "OdomEKF");
    OdomEKFTransform odomEKF;
    ros::Rate loop_rate(50);
    while(ros::ok()) {
        ros::spin();
    }
    return 0;
}