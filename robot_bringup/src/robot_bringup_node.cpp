#include "serialPort.hpp"
#include "canPort.hpp"
#include "contract.hpp"
#include "utilty.h"
#include "temp.hpp"
#include "encoder.hpp"
#include "usb2io.hpp"
#include "netRelay.hpp"
#include "ultrasonic.hpp"

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Range.h>
#include <robot_bringup/vel.h>
#include <robot_bringup/classis_status.h>
#include <robot_bringup/joint.h>
#include <robot_bringup/get_tof_distance.h>
#include <robot_bringup/set_odom_zero.h>

#include <iostream>
#include <thread>
#include <boost/thread.hpp>
#include <mutex>
#include <arpa/inet.h>
#include <string>

std::mutex m_lock;
std::mutex rs485_lock;
// std::mutex direction_lock;

bool _directionFlag = false; // false=向前  true = 向后   默认向前

ros::Publisher 	markCompleted_pub;

nav_msgs::Odometry::Ptr odom_wheel;

boost::array<double,36UL> ODOM_POSE_COVARIANCE = {	1e-3, 0, 0, 0, 0, 0, 
													0, 1e-3, 0, 0, 0, 0,
													0, 0, 1e6, 0, 0, 0,
													0, 0, 0, 1e6, 0, 0,
													0, 0, 0, 0, 1e6, 0,
													0, 0, 0, 0, 0, 1e3};
boost::array<double,36UL> ODOM_POSE_COVARIANCE2 = {	1e-9, 0, 0, 0, 0, 0, 
													0, 1e-3, 1e-9, 0, 0, 0,
													0, 0, 1e6, 0, 0, 0,
													0, 0, 0, 1e6, 0, 0,
													0, 0, 0, 0, 1e6, 0,
													0, 0, 0, 0, 0, 1e-9};

boost::array<double,36UL> ODOM_TWIST_COVARIANCE = {	1e-3, 0, 0, 0, 0, 0, 
													0, 1e-3, 0, 0, 0, 0,
													0, 0, 1e6, 0, 0, 0,
													0, 0, 0, 1e6, 0, 0,
													0, 0, 0, 0, 1e6, 0,
													0, 0, 0, 0, 0, 1e3};
boost::array<double,36UL> ODOM_TWIST_COVARIANCE2 = {1e-9, 0, 0, 0, 0, 0, 
													0, 1e-3, 1e-9, 0, 0, 0,
													0, 0, 1e6, 0, 0, 0,
													0, 0, 0, 1e6, 0, 0,
													0, 0, 0, 0, 1e6, 0,
													0, 0, 0, 0, 0, 1e-9};


void classis_cmd_package(uint8_t Node_ID, int8_t CMD, int16_t data1, int16_t data2, uint8_t Byte6, uint8_t Byte7, canPort &myserial)
{
	int ret;
	ClassisRecvCmd_t senddata;
	senddata.cmdstruct.Node_ID = Node_ID;
	senddata.cmdstruct.CMD = CMD;
	senddata.cmdstruct.data1.data16 = data1;
	senddata.cmdstruct.data2.data16 = data2;
	senddata.cmdstruct.Byte6 = Byte6;
	senddata.cmdstruct.Byte7 = Byte7;
	// std::cout << "测试200mm/s" << std::setw(2) << std::hex << (unsigned int)(unsigned char)uartsenddata.cmddata[0] << (unsigned int)(unsigned char)uartsenddata.cmddata[1] << (unsigned int)(unsigned char)uartsenddata.cmddata[2] << (unsigned int)(unsigned char)uartsenddata.cmddata[3] << (unsigned int)(unsigned char)uartsenddata.cmddata[4] << (unsigned int)(unsigned char)uartsenddata.cmddata[5] << (unsigned int)(unsigned char)uartsenddata.cmddata[6] << (unsigned int)(unsigned char)uartsenddata.cmddata[7] << endl;
	ret = myserial.writeBuffer(1,senddata.cmddata, CLASSIS_CMD_LENGTH);
}

/**
  * @brief  KEYA 设备使能控制
  * @param
  * @retval None
  */
void KEYAActivatedModeEnable(uint32_t keyaid, KEYAACTIVE_t isactive, canPort& myserial)
{
	int ret;
	CANTxMsg_t txmsg;
	DATA32_t candata;
	txmsg.CANTxStruct.StdId = 0x600 + keyaid; //SDO COB-ID = 0x600 +ID;
	txmsg.CANTxStruct.DLC = 0x05;	//keya的使能使用5字节SDO的命令字 0x2F
	txmsg.CANTxStruct.Data[0] = 0x2F;	 //命令字0x22
	if (isactive == KEYADISABLE)
	{
		txmsg.CANTxStruct.Data[1] = 0x0C;  //对象名称 0x200C-00 紧急关机
		txmsg.CANTxStruct.Data[2] = 0x20;
	}
	else
	{
		txmsg.CANTxStruct.Data[1] = 0x0D;  //对象名称 0x200C-00 紧急关机释放
		txmsg.CANTxStruct.Data[2] = 0x20;
	}
	txmsg.CANTxStruct.Data[3] = 0x00;   //数据为0
	//通过data32类型进行转换
	candata.data32 = 0;
	txmsg.CANTxStruct.Data[4] = candata.raw[0];
	txmsg.CANTxStruct.Data[5] = candata.raw[1];
	txmsg.CANTxStruct.Data[6] = candata.raw[2];
	txmsg.CANTxStruct.Data[7] = candata.raw[3];
	//加入发送缓冲队列
	ret = myserial.writeBuffer(txmsg.CANTxStruct.StdId, txmsg.CANTxStruct.Data, txmsg.CANTxStruct.DLC);
}

/**
  * @brief  KEYA 目标转速（rpm） 参数设置(0-1000)
  * @param
  * @retval None
  */
void KEYATargetVelocitySet(uint32_t keyaid, uint8_t channel, uint32_t targetvelocity, canPort& myserial)
{
	int ret;
	CANTxMsg_t txmsg;
	DATA32_t candata;
	txmsg.CANTxStruct.StdId = 0x600 + keyaid; //SDO COB-ID = 0x600 +ID;
	txmsg.CANTxStruct.DLC = 0x08;	//keya速度控制使用8直接SDO的命令字 0x23
	txmsg.CANTxStruct.Data[0] = 0x23;	 //命令字0x23
	txmsg.CANTxStruct.Data[1] = 0x00;  //对象名称 0x2000-00
	txmsg.CANTxStruct.Data[2] = 0x20;

	txmsg.CANTxStruct.Data[3] = channel; //所有科亚驱动默认通道号1

	//通过data32类型进行转换
	candata.data32 = (uint32_t)targetvelocity;
	txmsg.CANTxStruct.Data[4] = candata.raw[0];
	txmsg.CANTxStruct.Data[5] = candata.raw[1];
	txmsg.CANTxStruct.Data[6] = candata.raw[2];
	txmsg.CANTxStruct.Data[7] = candata.raw[3];
	//加入发送缓冲队列
	ret = myserial.writeBuffer(txmsg.CANTxStruct.StdId, txmsg.CANTxStruct.Data, txmsg.CANTxStruct.DLC);
}

void can0rev_thread(canPort *myserial,const ros::Publisher &_pub, const ros::Publisher &_pub2, const double &wheelbase, nav_msgs::Odometry::Ptr odom_wheel) 
{
	robot_bringup::classis_status::Ptr status;
	status.reset(new robot_bringup::classis_status());
	static bool flag = false;
	int cnt = 0;

	while(ros::ok())
	{
		myserial->readBuffer();
		if(!(Classis::analy_data(&myserial->rxmsg, status, odom_wheel, wheelbase)))// 0x01 or 0x02
		{
			cnt++;
		}
		if(cnt%4 == 0)
		{
			odom_wheel->header.stamp = ros::Time::now();
			if(odom_wheel->twist.twist.linear.x == 0 && odom_wheel->twist.twist.angular.y ==0 && odom_wheel->twist.twist.angular.z == 0)
			{
				odom_wheel->pose.covariance = ODOM_POSE_COVARIANCE2;
				odom_wheel->twist.covariance = ODOM_TWIST_COVARIANCE2;
			}
			else
			{
				odom_wheel->pose.covariance = ODOM_POSE_COVARIANCE;
				odom_wheel->twist.covariance = ODOM_TWIST_COVARIANCE;				
			}
			_pub2.publish(odom_wheel);			
		}
		if(cnt%8 == 0)
		{
			cnt = 0;
			_pub.publish(status);
			status.reset(new robot_bringup::classis_status());
			//ROS_INFO("voltage:-----------%f\n",status->battery_voltage);
		}
	}
	return;
}

void getTemp_thread(modbus_t *ctx, const ros::Publisher &_pub) 
{
	std_msgs::Float64 temperature;
	temp temptrans(ctx);
	ros::Rate r(0.2);
	while(ros::ok())
	{
		std::unique_lock<std::mutex> lg(rs485_lock);
		temperature.data = temptrans.getTemperature();
		_pub.publish(temperature);
		lg.unlock();
		r.sleep();
	}
	return;
}

void getDis_thread(modbus_t *ctx, const ros::Publisher &_pub) 
{
	sensor_msgs::Range distance;
	ultrasonic uls(ctx);
	ROS_INFO("ultrasonic modbus object is created");
	ros::Rate r(5);
	distance.header.frame_id = "base_link";
	distance.radiation_type = sensor_msgs::Range::ULTRASOUND;
	distance.field_of_view = 1.047198;
	distance.min_range = 0.03;
	distance.max_range = 4.50;
	while(ros::ok())
	{
		distance.header.stamp = ros::Time::now();
		if(!_directionFlag)
			distance.range = (static_cast<float>(uls.getDistance(FRONT_ULTRA))) / 1000;
		else
			distance.range = (static_cast<float>(uls.getDistance(BACKWARD_ULTRA))) / 1000;
		_pub.publish(distance);
		r.sleep();
	}
	return;
}

void getJointDeg_Thread(modbus_t *ctx,const ros::Publisher &big_pub, const ros::Publisher &small_pub) 
{
	std_msgs::Float64 big_deg, small_deg;
	encoder joint(ctx);
	ros::Rate r(50);
	while(ros::ok())
	{
		std::unique_lock<std::mutex> lg(rs485_lock);
		big_deg.data = -joint.getAbsolutelyDeg(BIGJOINT_MODBUS_ADDR);
		usleep(100);
		small_deg.data = joint.getAbsolutelyDeg(SMALLJOINT_MODBUS_ADDR);
		big_pub.publish(big_deg);
		small_pub.publish(small_deg);
		lg.unlock();
		r.sleep();
	}
	modbus_close(ctx);
	modbus_free(ctx);
	return;
}

void singleVel_callback(const robot_bringup::vel::ConstPtr &single_vel,canPort &myserial)
{
	int v = (single_vel->vel_right + single_vel->vel_left) / 2;
	if(v >=0)
		_directionFlag = true;	//如果大于0，则是后退。
	else
		_directionFlag = false;	//如果小于0，则是前进。
	std::unique_lock<std::mutex> lg(m_lock);
	//ROS_INFO("LEFT:%d----RIGHT:%d",single_vel->vel_left,single_vel->vel_right);
	classis_cmd_package(0x01, 0x01, (int16_t)(single_vel->vel_right), (int16_t)(single_vel->vel_left), 0, 0, myserial);
	lg.unlock(); 
	ROS_INFO_STREAM("[singleVel_thread=" << boost::this_thread::get_id() << "]" << "LEFT- " << single_vel->vel_left << "RIGHT- " << single_vel->vel_right);
}

void mark_callback(const std_msgs::Int16::ConstPtr &msg, serialPort &myserial)
{
	std::string *str = new std::string("<DFlag1," + std::to_string(msg->data) + '>');
	myserial.writeBuffer((uint8_t *)(str->data()), str->length());
	int ret = Mark::analy_data(myserial, *str, str->length());
	if(ret)
	{
		ROS_INFO_STREAM("mark contract error, return wrong string ");
		std_msgs::Int32 databuf;
		databuf.data = -1;
		markCompleted_pub.publish(databuf);
		return;
	}
	usleep(5000);
	
	str->assign("<DFlag1,POSR,4>");	
	myserial.writeBuffer((uint8_t *)(str->data()), str->length());
	ret = Mark::analy_data(myserial, *str, str->length());
	if(ret)
	{
		ROS_INFO_STREAM("mark contract error, return wrong string ");
		std_msgs::Int32 databuf;
		databuf.data = -1;
		markCompleted_pub.publish(databuf);
		return;
	}
	usleep(5000);

	str->assign("<DFlag1,X,0>");	
	myserial.writeBuffer((uint8_t *)(str->data()), str->length());
	ret = Mark::analy_data(myserial, *str, str->length());
	if(ret)
	{
		ROS_INFO_STREAM("mark contract error, return wrong string ");
		std_msgs::Int32 databuf;
		databuf.data = -1;
		markCompleted_pub.publish(databuf);
		return;
	}
	usleep(5000);

	str->assign("<DFlag1,Y,0>");	
	myserial.writeBuffer((uint8_t *)(str->data()), str->length());
	ret = Mark::analy_data(myserial, *str, str->length());
	if(ret)
	{
		ROS_INFO_STREAM("mark contract error, return wrong string ");
		std_msgs::Int32 databuf;
		databuf.data = -1;
		markCompleted_pub.publish(databuf);
		return;
	}

	str->assign("<X>");	
	myserial.writeBuffer((uint8_t *)(str->data()), str->length());
	ret = Mark::analy_data(myserial, *str, str->length());
	if(ret)
	{
		ROS_INFO_STREAM("mark contract error, return wrong string ");
		std_msgs::Int32 databuf;
		databuf.data = -1;
		markCompleted_pub.publish(databuf);
		return;
	}

	sleep(2);
	ret = Mark::analy_data(myserial, "<XE>", 4);
	if(!ret)
	{
		ROS_INFO_STREAM("mark successfully");
		std_msgs::Int32 databuf;
		databuf.data = 1;
		markCompleted_pub.publish(databuf);
	}
	
	delete str;//
}

void vel_callback(const geometry_msgs::Twist::ConstPtr &cmd_vel, const double &wheelbase, canPort &myserial)
{
	// static double last_v_l = 0;
	// static double last_v_r = 0;
	double v = cmd_vel.get()->linear.x;	 // 线速度
	double w = cmd_vel.get()->angular.z; // 角速度
	double l = wheelbase;				 // m 两轮间距
	//std::cout << wheelbase << std::endl;
	// double r = 0.075;                    // m	轮子半径
	double v_l = -v + w * l / 2; // 左轮速度 m/s
	double v_r = -v - w * l / 2; // 右轮速度 m/s

	/*速度低通滤波，起到缓冲作用*/
	// double final_v_l = v_l * 0.4 + last_v_l* 0.6;
	// double final_v_r = v_r * 0.4 + last_v_r* 0.6;
	// ROS_INFO("\033[1;32m vl = %.6lf vr = %.6lf\033[0m", v_l, v_r);
	// ROS_INFO("\033[1;32m final_vl = %.6lf final_vr = %.6lf\033[0m", final_v_l, final_v_r);
	// last_v_l = final_v_l;
	// last_v_r = final_v_r;

	std::unique_lock<std::mutex> lg(m_lock);
	usleep(1000);
	classis_cmd_package(0x01, 0x01, (int16_t)(v_l * 1000), (int16_t)(v_r * 1000), 0, 0, myserial);
	// KEYATargetVelocitySet(1, 2, (int16_t)(v_l * 1000), myserial);
	// KEYATargetVelocitySet(1, 1, (int16_t)(v_r * 1000), myserial);
	lg.unlock(); 
	//ROS_INFO_STREAM("[vel_thread=" << boost::this_thread::get_id() << "]");
}

static serialPort *mark_serial;
static USB2GPIO *GPIO;
//modbus_t *ctx_tcp = modbus_new_tcp("192.168.1.232",10000);
//netRelay *netrelay = new netRelay(ctx_tcp);

void fanControllCallback(const std_msgs::Bool::ConstPtr &msg)
{
	if(msg->data == true)
		GPIO->GPIO_WrirePin(0,1);
	else
		GPIO->GPIO_WrirePin(0,0);
}

void heatControllCallback(const std_msgs::Bool::ConstPtr &msg)
{
	if(msg->data == true)
		GPIO->GPIO_WrirePin(1,1);
	else
		GPIO->GPIO_WrirePin(1,0);
}

bool set_odom_zero_callback(robot_bringup::set_odom_zero::Request& req, robot_bringup::set_odom_zero::Response& res)
{
	//odom_wheel.reset(new nav_msgs::Odometry());
	odom_wheel->pose.pose.position.x = 0;
	odom_wheel->pose.pose.position.y = 0;
	odom_wheel->pose.pose.position.z = 0;

	odom_wheel->pose.pose.orientation.w = 1;
	odom_wheel->pose.pose.orientation.x = 0;
	odom_wheel->pose.pose.orientation.y = 0;
	odom_wheel->pose.pose.orientation.z = 0;

	theta = 0.0;

	res.result = true;
	return true;
}

void siginthandler(int sig)
{
    ROS_INFO("shutting down!");
	delete GPIO;
	//delete netrelay;
    ros::shutdown();
    exit(0);
}

int main(int argc, char **argv)
{
	std::string uart_id, topic;
	double wheelbase;

	mark_serial = new serialPort;
	canPort mycan;
	modbus_t* ctx = modbus_new_rtu("/dev/ttyTHS4", 115200, 'N', 8, 1);// "/dev/Modbus-485" encoder
	modbus_t* ctx_ultra = modbus_new_rtu("/dev/Modbus-485", 9600, 'N', 8, 1);//"/dev/ultrasonic"
	modbus_rtu_set_serial_mode(ctx,MODBUS_RTU_RS485);
	modbus_rtu_set_rts(ctx , MODBUS_RTU_RTS_DOWN);
	modbus_rtu_set_rts_delay(ctx,200);

	// modbus_set_debug(ctx_ultra,true);// 开启modbus debug

	//设置超声波485通讯的超时时间为1s 否则测量需要500ms modbus库会报超时
	uint32_t secbuf = 1;
	uint32_t usecbuf = 0;
	modbus_set_response_timeout(ctx_ultra, secbuf, usecbuf);

	ros::init(argc, argv, "robot_bringup_node");
	ros::NodeHandle nh /*("~")*/;

	nh.param<std::string>("robot_bringup_node/vel_topic", topic, "/cmd_vel");
	nh.param<double>("robot_bringup_node/wheelbase", wheelbase, 0.5);

	printf("serialPort %s Opening\n", uart_id.c_str());

	mark_serial->OpenPort("/dev/ttyTHS0");
	mark_serial->setup(9600, 0, 8, 1, 'N');
	mycan.openPort("can0");//

	nav_msgs::Odometry odom_wheel1;
	
	odom_wheel.reset(new nav_msgs::Odometry());
	odom_wheel->child_frame_id = "base_link";
	odom_wheel->header.seq = 1;
	odom_wheel->header.frame_id = "odom";

	GPIO = new USB2GPIO;
	//netrelay->setNetRelay(SLAVE_ID,RELAY_ON);

	ros::Subscriber vel_sub = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 100, boost::bind(&vel_callback, _1, wheelbase, mycan));
	ros::Subscriber singleVel_sub = nh.subscribe<robot_bringup::vel>("/single_vel", 100, boost::bind(&singleVel_callback, _1, mycan));
	ros::Subscriber mark_sub = nh.subscribe<std_msgs::Int16>("/mark", 100, boost::bind(&mark_callback, _1, *mark_serial));
	ros::Subscriber fan_sub = nh.subscribe<std_msgs::Bool>("/fan_flag", 10, fanControllCallback, ros::TransportHints().tcpNoDelay());
	ros::Subscriber heat_sub = nh.subscribe<std_msgs::Bool>("/heat_flag", 10, heatControllCallback, ros::TransportHints().tcpNoDelay());

	ros::Publisher 	classis_status_pub = nh.advertise<robot_bringup::classis_status>("/classis_status",50);
	ros::Publisher 	internal_temperature_pub = nh.advertise<std_msgs::Float64>("/internal_temperature",50);
	ros::Publisher 	big_joint_deg_pub = nh.advertise<std_msgs::Float64>("/big_joint_deg",1);
	ros::Publisher 	small_joint_deg_pub = nh.advertise<std_msgs::Float64>("/small_joint_deg",1);
	ros::Publisher 	tof_distance_pub = nh.advertise<std_msgs::Float64>("/tof_distance", 1);
	ros::Publisher 	ultrasonicDistance_pub = nh.advertise<sensor_msgs::Range>("/ultrasonic_distance", 1);
	ros::Publisher 	odom_pub = nh.advertise<nav_msgs::Odometry>("/odom_wheel", 10);
	markCompleted_pub = nh.advertise<std_msgs::Int32>("/markcompleted", 10);
	ros::ServiceServer server2 = nh.advertiseService("set_odom_zero", set_odom_zero_callback);

	ros::AsyncSpinner s(4);
 	s.start();

	std::thread classisControl_thread(can0rev_thread, &mycan, classis_status_pub, odom_pub, wheelbase, odom_wheel);
	// std::thread temperatureThread(getTemp_thread, ctx, internal_temperature_pub);
	std::thread distanceThread(getDis_thread, ctx_ultra, ultrasonicDistance_pub);
	std::thread jointThread(getJointDeg_Thread, ctx, big_joint_deg_pub, small_joint_deg_pub);
	classisControl_thread.detach();
	distanceThread.detach();
	// temperatureThread.detach();
	jointThread.detach();

	signal(SIGINT, siginthandler);

	// KEYAActivatedModeEnable(1, KEYAENABLE, mycan);

	ros::Rate r(1);
	while (ros::ok())
 	{
		// ROS_INFO_STREAM("Main thread [" << boost::this_thread::get_id() << "].");
		r.sleep();
  	}

	mark_serial->ClosePort();
	delete mark_serial;
	return 0;
}

