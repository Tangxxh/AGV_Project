/*
 * @Author: Wei Zhifei
 * @Date: 2022-12-15 13:18:38
 */
#include "contract.hpp"
#include <tf/tf.h>
#include <cmath>

double theta;

int Classis::analy_data(CANTxMsg_t *msg, robot_bringup::classis_status::Ptr ptrTopicMsg, nav_msgs::Odometry::Ptr odom, double wheelbase)
{
    ClassisRecvCmd_t recvdata;
    double v_l, v_r;
    double l = wheelbase;
    double wheel_radius = 0.32;

    tf::Quaternion q;
    double dx;

    //double 
	memcpy(recvdata.cmddata, msg->CANTxStruct.Data, 8);

	// printf("%x,%x,%x,%x\n",recvdata.cmdstruct.Node_ID,recvdata.cmdstruct.CMD,
	// 									recvdata.cmdstruct.Byte6,recvdata.cmdstruct.Byte7);
	if(recvdata.cmdstruct.Node_ID == 0x02)
	{
		switch (recvdata.cmdstruct.CMD)
		{
		case 0x01:
            ptrTopicMsg->battery_voltage = ((double)(recvdata.cmdvolstruct.voltage))/10;
            v_l = ((double)(recvdata.cmdvolstruct.left_front_vel.data16))/1000;
            v_r = ((double)(recvdata.cmdvolstruct.right_front_vel.data16))/1000;
            odom->twist.twist.linear.x = -0.5 * (v_l + v_r) * cos(theta);
            odom->twist.twist.linear.y = -0.5 * (v_l + v_r) * sin(theta);
            odom->twist.twist.angular.z = (v_l - v_r) / l;
			break;
        case 0x02:
            if((recvdata.cmdstatestruct.Byte6)&0x01)
            {
                ptrTopicMsg->emergency_stop = true; // already not used
            }
            if((recvdata.cmdstatestruct.Byte6)&0x02)
            {
                ptrTopicMsg->voltage_fault = true;
            }
            if((recvdata.cmdstatestruct.Byte6)&0x04)
            {
                ptrTopicMsg->motor_fault= true;
            }
            if((recvdata.cmdstatestruct.Byte6)&0x08)
            {
                ptrTopicMsg->back_touch = true;
            }
            if((recvdata.cmdstatestruct.Byte6)&0x10)
            {
                ptrTopicMsg->front_touch = true;
            }
            if((recvdata.cmdstatestruct.Byte6)&0x20)
            {
                ptrTopicMsg->backward_deny = true;
            }
            if((recvdata.cmdstatestruct.Byte6)&0x40)
            {
                ptrTopicMsg->forward_deny= true;
            }
            break;
        case 0x03:  //仅计算前轮
            dx = ((double)(recvdata.cmdfrontencoderstruct.left_front_encoder.data16 \
                                                    + recvdata.cmdfrontencoderstruct.right_front_encoder.data16))\
                                                    /8000.0 * 0.30 * M_PI / 50;
            theta -= ((double)(recvdata.cmdfrontencoderstruct.left_front_encoder.data16 \
                                                    - recvdata.cmdfrontencoderstruct.right_front_encoder.data16))\
                                                    /8000 * 0.30 * M_PI / l / 50;
            odom->pose.pose.position.x += dx * cos(theta);
            odom->pose.pose.position.y += dx * sin(theta);
            q.setRPY(0,0,theta);
            odom->pose.pose.orientation.w = q.getW();
            odom->pose.pose.orientation.x = q.getX();
            odom->pose.pose.orientation.y = q.getY();
            odom->pose.pose.orientation.z = q.getZ();

            // std::cout<< "轮子减速比：" << recvdata.cmdfrontencoderstruct.Byte6 << std::endl;
            break;
        case 0x04:
            break;
		
		default:
            return -1;
			break;
		}
	}
    return 0;
}

double Laser::analy_data(serialPort &myserial)
{
    int16_t len, i;
    int16_t distance;  //距离
    uint8_t recvdata;		// tcp recv a char in queque
	uint8_t uartrecvstat = UART_HEAD1;	//接收状态
	uint8_t uartlen = 0;				//已经接收的长度
	uint8_t uartcheckxor = 0;			//校验和

    uint8_t read_databuff[10];
    uint8_t databuff[6];

    uint8_t str[6];
    str[0] = 0x02;
    str[1] = 0x43;
    str[2] = 0xb0;
    str[3] = 0x01;
    str[4] = 0x03;
    str[5] = str[1]^str[2]^str[3];

    myserial.writeBuffer(str,6);
    usleep(1000);
    len = myserial.readBuffer(read_databuff,10);
    for (i=0; i< len ; i++)
    {
        recvdata = read_databuff[i];
        switch ( uartrecvstat )
        {
            case UART_HEAD1:
                if( recvdata == 0x02)
                {
                    uartrecvstat = UART_HEAD2;
                }              
                break;
            case UART_HEAD2:
                if( recvdata == 0x06 )
                {
                    uartrecvstat = UART_DATA;
                }
                else if( recvdata == 0x02 )
                {
                    uartrecvstat = UART_HEAD2;
                }
                else
                {
                    uartrecvstat = UART_HEAD1;
                }
                break;
            case UART_DATA:
                databuff[uartlen] = recvdata;
                uartlen++;
                if(uartlen >= 3){
                    uartrecvstat = UART_CHACK;  
                }
                break;
            case UART_CHACK:
                uartcheckxor = 0x06 ^ databuff[0] ^ databuff[1];
                if(uartcheckxor == recvdata)
                {
                    distance = databuff[0] << 8 | databuff[1];
                }
                uartrecvstat = UART_HEAD1;
                break;
            default:
                uartrecvstat = UART_HEAD1;
                break;
        }
    }    
    return static_cast<double> (distance);
}

/* 输入：串口对象，需要对比的字符串
*  输出：0：成功通讯，1：有错误  未加超时选项可能会阻塞线程
*/
int Mark::analy_data(serialPort &myserial, std::string str, int length)
{
    uint8_t buffer[64];
    // usleep(1000);
    int len = myserial.readBuffer(buffer, length);
    buffer[length] = '\0';
    std::string rev = (char*)buffer;
    if(str == rev)
        return 0;
    else
    {
        std::cout << "--------------- " << rev << "--------------- " << std::endl;
        return -1;
    }
}