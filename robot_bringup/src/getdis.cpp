#include "utilty.h"
#include "tofSerialPort.hpp"

#include <ros/ros.h>
#include <std_msgs/Float64.h>

#include <arpa/inet.h>

static tofSerialPort *myserial;
ros::Publisher _distance_pub;

bool get_distance(void)
{   
	TOF_Msg_t tof_tx_buf;
	TOF_Msg_t tof_rx_buf;
	std_msgs::Float64 tof_distance;

	/*TOF*/
	tof_tx_buf.TOF_TX_STR.STX = 0x02;
	tof_tx_buf.TOF_TX_STR.ETX = 0x03;
	tof_tx_buf.TOF_TX_STR.command = 0x43;
	tof_tx_buf.TOF_TX_STR.data.raw[0] = 0xB0;
	tof_tx_buf.TOF_TX_STR.data.raw[1] = 0x01;

	tof_tx_buf.TOF_TX_STR.BCC = tof_tx_buf.TOF_TX_Msg[1] ^ tof_tx_buf.TOF_TX_Msg[2] ^ tof_tx_buf.TOF_TX_Msg[3];
	// printf("[send]: %x %x %x %x %x %x\n",tof_tx_buf.TOF_TX_Msg[0],tof_tx_buf.TOF_TX_Msg[1],tof_tx_buf.TOF_TX_Msg[2],tof_tx_buf.TOF_TX_Msg[3],tof_tx_buf.TOF_TX_Msg[4],tof_tx_buf.TOF_TX_Msg[5]);
	myserial->writeBuffer(tof_tx_buf.TOF_TX_Msg, 6);
	int ret = myserial->readBuffer(tof_rx_buf.TOF_TX_Msg,6);
	if(ret)
	{
		if (tof_rx_buf.TOF_TX_STR.command == 0x06 && tof_rx_buf.TOF_TX_STR.BCC == tof_rx_buf.TOF_TX_Msg[1] ^ tof_rx_buf.TOF_TX_Msg[2] ^ tof_rx_buf.TOF_TX_Msg[3])
		{
			tof_distance.data = (double)((int16_t)ntohs(tof_rx_buf.TOF_TX_STR.data.data16))/10;//转换字节序，大小端转换，并换算成浮点数。
			_distance_pub.publish(tof_distance);
		}
		else
		{
			// printf("[recv %d ]: %x %x %x %x %x %x\n", ret, tof_rx_buf.TOF_TX_Msg[0],tof_rx_buf.TOF_TX_Msg[1],tof_rx_buf.TOF_TX_Msg[2],tof_rx_buf.TOF_TX_Msg[3],tof_rx_buf.TOF_TX_Msg[4],tof_rx_buf.TOF_TX_Msg[5]);
			ROS_INFO("arm laser contract error");
			return false; 
		}
	}
	else
	{
		ROS_ERROR("laser not recv!");
		return false;
	}
	
    return true;
}

int main(int argc, char** argv)
{
	myserial = new tofSerialPort;
	std::string uart_id;

	ros::init(argc, argv, "get_dis_node");
	ros::NodeHandle nh /*("~")*/;

	nh.param<std::string>("getdis/uart_id", uart_id, "/dev/ttyUSB1");
	_distance_pub = nh.advertise<std_msgs::Float64>("/tof_distance", 1);

	myserial->OpenPort(uart_id.c_str());
	myserial->setup(115200, 0, 8, 1, 'N');

	ros::Rate r(5);
	while(ros::ok())
	{
		get_distance();
		r.sleep();
	}

	myserial->ClosePort();
	delete myserial;
	return 0;
}