
#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

#include <stdlib.h>
#include <stdio.h>
#include <sys/time.h>
#include <signal.h>
#include <unistd.h>
#include <eigen3/Eigen/Geometry>
#include <std_msgs/Bool.h>
#include <mutex>
using std::recursive_mutex;
using namespace std;
struct TransOdom
{
	TransOdom ( )
	{
	}
    tf::TransformBroadcaster transform;
	void Publish ( double  XIntegral , double YIntegral , double  AngleIntegral )
	{
		geometry_msgs::TransformStamped odom_trans;
		odom_trans.header.stamp = ros::Time::now ( );;
		odom_trans.header.frame_id = "odom";
 		odom_trans.child_frame_id = "base_footprint_2";
		odom_trans.transform.translation.x = XIntegral;
		odom_trans.transform.translation.y = YIntegral;
		odom_trans.transform.translation.z = 0.0;
		odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw ( AngleIntegral );
		transform.sendTransform ( odom_trans );
	}
};
//将tfq转换为rpy
class imu_odom
{
private:
    /* data */
    ros::NodeHandle n;
    ros::Subscriber imuSub;
    ros::Subscriber simSub;
    ros::Subscriber mclSub;
    ros::Subscriber rtkSub;
    ros::Subscriber ekfSub;
    ros::Subscriber odomSub;
    ros::Publisher odom_new_pub;
    ros::Publisher odom_new_pub3;
    recursive_mutex lock;
    ros::Time now_time ;
    ros::Time incress_time_nec;
    tf::Vector3 vw;
    tf::Vector3 pw;
    nav_msgs::Odometry odom_pub;
    tf::Quaternion qw;
    nav_msgs::Odometry amcl_pose;
    tf::Pose amcl;
    nav_msgs::Odometry amcl_q;
    nav_msgs::Odometry last_pose;
    geometry_msgs::Quaternion angle_imu;
    bool first_imu_recv = false;
    bool first_amcl = true;
    bool locState_ = true;
    bool init_flag = false;
    double amcl_time = 0.0;
    double XIntegral = 0.0, YIntegral = 0.0, line = 0.0, angle = 0.0;
    double wz_imu ;
    double vx=0.0 ;
    double vy=0.0 ;
    double vz=0.0 ;
    double last_imu_angerV=0.0;
    double last_time = 0.0;
    double dwz = 0.0;
    TransOdom trans;
    bool use_imu_noise_biase = false;
    bool v_receive_flag = false;
    //void odomCallback(const nav_msgs::Odometry::ConstPtr &odom);
    //void imuCallback(const sensor_msgs::Imu::ConstPtr & imu_msg);
public:
    imu_odom(ros::NodeHandle &nh)
    {
        n = nh;
        ros::NodeHandle n( "~" );
	    odom_new_pub = n.advertise<nav_msgs::Odometry> ( "/odom_2" , 1 );
	    odom_new_pub3 = n.advertise<nav_msgs::Odometry> ( "/odom_3" , 1 );
        imuSub = n.subscribe<sensor_msgs::Imu> ( "/imu/data" , 5 , boost::bind ( &imu_odom::imuCallback , this , _1 ) );
	    simSub = n.subscribe<std_msgs::Bool> ( "/localization_status" , 1 , boost::bind ( &imu_odom::LocStateCallback , this , _1 ) );
	    //rtkSub = n.subscribe<nav_msgs::Odometry> ( "/odometry/UTM" , 1 , boost::bind ( &imu_odom::rtkCallBack , this , _1 ) );
        // mclSub = n.subscribe<nav_msgs::Odometry> ( "/ndt_pose_odom" , 1 , boost::bind ( &imu_odom::amclCallBack , this , _1 ) );
        odomSub = n.subscribe<nav_msgs::Odometry>("/odom_wheel",100,boost::bind ( &imu_odom::odomCallback , this , _1 ));
        ekfSub = n.subscribe<nav_msgs::Odometry>("/odom",100,boost::bind ( &imu_odom::ekfCallback , this , _1 ));
        odom_pub.header.frame_id = "odom";
		odom_pub.child_frame_id = "odom_1"; //"base_link";
		odom_pub.pose.covariance [ 0 ] = 0.01;
		odom_pub.pose.covariance [ 7 ] = 0.01;
		odom_pub.pose.covariance [ 14 ] = 99999;
		odom_pub.pose.covariance [ 21 ] = 99999;
		odom_pub.pose.covariance [ 28 ] = 99999;
		odom_pub.pose.covariance [ 35 ] = 0.01;
		odom_pub.twist.covariance [ 0 ] = 999999;
		odom_pub.twist.covariance [ 7 ] = 999999;
		odom_pub.twist.covariance [ 14 ] = 999999;
		odom_pub.twist.covariance [ 21 ] = 999999;
		odom_pub.twist.covariance [ 28 ] = 999999;
		odom_pub.twist.covariance [ 35 ] = 999999;
    }

    void imuCallback(const sensor_msgs::Imu::ConstPtr & imu_msg)
    {
        lock.lock ( );
        wz_imu = imu_msg->angular_velocity.z;
        angle_imu = imu_msg->orientation;
        //cout<<"imu_angular_velocity z : "<<wz_imu<<endl;
        first_imu_recv = true;
        lock.unlock ( );
    }
    void LocStateCallback(const std_msgs::BoolConstPtr& msg)
	{
        lock.lock ( );
		locState_ = msg->data;
        //cout<<"enter LocStateCallback "<<endl<<endl;;
        
        lock.unlock ( );
	}

    // void amclCallBack(const nav_msgs::Odometry::ConstPtr &amcl)
    // {
    //     lock.lock ( );
    //     if (locState_)
    //     {
    //         amcl_pose = *amcl;
    //         amcl_q.pose.pose.orientation = amcl->pose.pose.orientation;
    //         amcl_time = amcl->header.stamp.toSec(); 
    //         cout<<"amcl_time:"<<amcl_time<<endl;
    //         cout<<"amcl_pose:x,y,z,yaw "<<amcl_pose.pose.pose.position.x<<","<<amcl_pose.pose.pose.position.y<<","<<amcl_pose.pose.pose.position.z<<endl<<endl;
    //         first_amcl = false;
    //     }
    //     lock.unlock ( );
    // }
    // void rtkCallBack(const nav_msgs::Odometry::ConstPtr &rtk_msg)
    // {
    //     lock.lock ( );
    //     if (locState_)
    //     {
    //         amcl_pose.position = rtk_msg->pose.pose.position;            
    //         amcl_q.orientation = rtk_msg->pose.pose.orientation;
    //         amcl_time = rtk_msg->header.stamp.toSec(); 
    //         //cout<<"amcl_time:"<<amcl_time<<endl;
    //         //cout<<"amcl_pose:x,y,z,yaw "<<amcl_pose.position.x<<","<<amcl_pose.position.y<<","<<amcl_pose.position.z<<endl;
    //         first_amcl = false;
    //     }
    //     lock.unlock ( );
    // }
    void Publish ( )
	{
		odom_new_pub.publish ( odom_pub );
	}
    void PubTrans(double x,double y,double yaw)
	{
		trans.Publish ( x , y , yaw );
	}
    void SetOdom( tf::Vector3 &pw,  tf::Quaternion &qw, tf::Vector3 &vw)
    {
        lock.lock ( );
        odom_pub.header.stamp = incress_time_nec;
        odom_pub.pose.pose.position.x = pw.getX();
        odom_pub.pose.pose.position.y = pw.getY();
        odom_pub.pose.pose.position.z = pw.getZ();
        odom_pub.pose.pose.orientation.w = qw.getW();
        odom_pub.pose.pose.orientation.x = qw.getX();
        odom_pub.pose.pose.orientation.y = qw.getY();
        odom_pub.pose.pose.orientation.z = qw.getZ();
        odom_pub.twist.twist.linear.x = vx;
        odom_pub.twist.twist.angular.z = wz_imu;
        lock.unlock ( );

    }
    void ekfCallback(const nav_msgs::Odometry::ConstPtr &ekfodom)
    {
        lock.lock ( );
        //first_odom = false;        
        //cout<<"first_imu:"<<first_imu<<", first_amcl:"<<first_amcl<<",locState_:"<<locState_<<endl;;
        if(first_imu_recv&& locState_ && v_receive_flag)
        {
            //初始化
            // last_pose = amcl_pose; 
            // nav_msgs::Odometry last_q = amcl_q;
            qw.setX(ekfodom->pose.pose.orientation.x);
            qw.setY(ekfodom->pose.pose.orientation.y);
            qw.setZ(ekfodom->pose.pose.orientation.z);
            qw.setW(ekfodom->pose.pose.orientation.w);
            pw.setX(ekfodom->pose.pose.position.x);
            pw.setY(ekfodom->pose.pose.position.y);
            pw.setZ(ekfodom->pose.pose.position.z);
            cout<<"init qw:"<<qw.x()<<","<<qw.y()<<","<<qw.z()<<","<<qw.w()<<endl;
            cout<<"init pw:"<<pw.x()<<","<<pw.y()<<","<<pw.z();
            //last_imu_angerV = wz_imu;
            last_time  = ekfodom->header.stamp.toSec();
            cout<<"last_time:"<<last_time<<endl;
            init_flag = true;
        }
        if(init_flag && !locState_ )
        {
            //(1)用四元数乘法来做积分 qw，vw,pw
            tf::Quaternion dq;
            //double dt = 0.02;
            double dx =0;
            double dy =0;
            dwz = wz_imu;
            if(std::isnan(dx) || std::isnan(dy) || std::isnan(dwz))
            {
                //cout<< "dx dy dwz is nan.....throw it ..."<<endl; 
                return;
            }
            double incress_time =  ekfodom->header.stamp.toSec() ;
            incress_time_nec = ekfodom->header.stamp;
            double dt =  (incress_time - last_time);
            dq.setRPY(dx*dt,dy*dt,dwz*dt);
            cout<<"dwz,dt,dwz*dt:"<<dwz<<","<<dt<<","<<dwz*dt<<endl;
            //cout<<"dq:x,y,z,yaw "<<dq.getX()<<","<<dq.getY()<<","<<dq.getZ()<<","<<tf::getYaw(dq)<<endl;                  
            cout<<"qw:yaw "<<tf::getYaw(qw)<<endl;                  
            
            qw = qw*dq;
            cout<<"qw*dq:yaw "<<tf::getYaw(qw)<<endl;  
            vw.setX(vx);
            vw.setY(vy);
            vw.setZ(vz);           
            vw = vw.rotate(qw.getAxis(),qw.getAngle());
            //cout<<"vw "<<vw.x()<<","<<vw.y()<<","<<vw.z()<<endl;

            //cout<<"pw old:"<<pw.x()<<","<<pw.y()<<","<<pw.z()<<endl;

            pw = pw + vw*dt*0.95;// +0.5*a*dt*dt; //a*dt*dt  
            
            //cout<<"pw new:"<<pw.x()<<","<<pw.y()<<","<<pw.z()<<endl;
                   
            last_time = incress_time;
            SetOdom( pw, qw, vw); 
            Publish();
            PubTrans( pw.getX(),pw.getY(), tf::getYaw(qw));

        }
        if (locState_  && !init_flag)
        {
            cout<<"locState_ :true ,init fail. "<<endl;
            return;           
        }
                       
        lock.unlock ( );
    } 

    void odomCallback(const nav_msgs::Odometry::ConstPtr &odom)
    { 
        lock.lock ( );
        vx = odom->twist.twist.linear.x;
        vy = odom->twist.twist.linear.y;
        vy = odom->twist.twist.linear.z;
        // nav_msgs::Odometry odom_new;
        // odom_new.pose = odom->pose;
        // odom_new.header.stamp = ros::Time::now ( );;
		// odom_new.header.frame_id = "odom";
        // odom_new.pose.pose.orientation = angle_imu;
        // odom_new_pub3.publish ( odom_new );
        // //cout<<"receive odom vx..."<<endl;
        v_receive_flag = true;
        lock.unlock ( );
    }
     ~imu_odom()
    {

    }
};


int main(int argc, char**argv)
{
    ros::init(argc,argv,"imu_odom_test");
    ros::NodeHandle nh("~");
    imu_odom imu_odom_(nh);
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
        
    }
    return 0;
}
