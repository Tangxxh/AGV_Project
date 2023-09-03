#include "canPort.hpp"

canPort::canPort()
{
    fd = -1;
}

/**********************
 *  创建初始化can接口
 * 参数：
 * 		name：can0 或者 can1
 * return：
 * 		套接字句柄
 *
 * *******************/
int canPort::openPort(const char *dev)
{
	struct sockaddr_can addr;

	struct ifreq ifr;	

	//创建socket
	if((this->fd = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
		perror("Error while creating CAN:socket");
		return -1;
	}
	
	strcpy(ifr.ifr_name, dev);
    if (ioctl(this->fd, SIOCGIFINDEX, &ifr) < 0) {
        perror("ioctl failed");
        return -1;
    }
	
	addr.can_family  = PF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;

//	setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0); //禁用过滤器

	if (bind(this->fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) { //绑定套接字 就是 套接字和canbus外设进行绑定
		perror("bind failed!");
		return -1;
	}
    printf("%s init successfully!\r\n",dev); 

	return this->fd;
}


/***************************************
 * can发送数据
 * arg：	fd：套接字句柄
 * 			id：can id，标准id
 * 			data：要传输的数据指针
 * 			len：数据长度
 * return：发送的字节数
 * 
 * ************************************/
int canPort::writeBuffer(unsigned int id, uint8_t *data, unsigned int len)
{
	int i,ret;

	struct can_frame frame;

	if((id > 0x7ff) || (len > 8)){
		perror("canbus id or data len error!");
        return -1;
	}

	frame.can_id = (canid_t)id;
	frame.can_dlc = len;
	for(i = 0; i < len; i++)
		frame.data[i] = data[i];
	
	ret = write(this->fd,&frame,sizeof(frame));
	if(ret < 0){
		perror("data write error!");
	}

	return ret;

}

/***************************************
 * can接收数据
 * arg：	fd：套接字句柄
 * 			id：can id，标准id
 * 			data：要传输的数据指针
 * 			len：数据长度
 * return：发送的字节数
 * 
 * ************************************/
int canPort::readBuffer(void)
{
	int len,i;
	struct can_frame buff;	
	//printf("can receive data!\n");
	len = read(fd,&buff,sizeof(buff));
	if( len == sizeof(buff))
	{
		rxmsg.CANTxStruct.StdId = buff.can_id;
		rxmsg.CANTxStruct.DLC = buff.can_dlc;
		for( i = 0 ; i < buff.can_dlc ; i++)
		{
			rxmsg.CANTxStruct.Data[i] = buff.data[i];
			//printf("%x ",buff.data[i]);
			
		}
		//printf("\n");
		//printf("id:%x,len:%x\n",rxmsg.CANTxStruct.StdId,rxmsg.CANTxStruct.DLC);
	}else{
		printf("CAN receive error!!!\n");
		return -1; //接收错误
	}

	return 0;
	
}

