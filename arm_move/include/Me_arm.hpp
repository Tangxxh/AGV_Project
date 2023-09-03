#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>

#include <linux/can.h>
#include <linux/can/raw.h>


#include <net/if.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

#define BIG_LR  0xC1
#define SCRAP   0xC2
#define OUTIN	0xC3
typedef enum
{
    position_mode = 0x01,
    Velocity_mode = 0x03,
}ELMO_mod;

#define CAN_CMD_LENGTH 		28		//4*5 + 8(0~8) 
// CAN sendcmd struct


class Me_arm
{
private:
    int fd;	//套接字返回句柄
    typedef union
    {
        uint8_t	raw[4];
        uint32_t data32;
    }DATA32_t;

    typedef union
    {
        uint8_t	raw[4];
        float_t data32;
    }FLOAT32_t;

    typedef union
    {
    	uint8_t	CANTxMsg[CAN_CMD_LENGTH];
    	struct
    	{
    		uint32_t    StdId;
    		uint32_t	ExtId;
    		uint32_t	IDE;
    		uint32_t	RTR;
    		uint32_t	DLC;
    		uint8_t		Data[8];
    	}CANTxStruct;
    }CANTxMsg_t;

    CANTxMsg_t rxmsg;
        
    struct can_filter{
        canid_t can_id;
        canid_t can_mask;
    };

    struct can_filter rfilter;
public:
    Me_arm(/* args */);
    ~Me_arm();

    typedef enum
    {
        FREE = 0x00,
        ACCLERATING,
        RUNNING,
        SLOWING
    }Stepper_Running_Status;

    typedef enum
    {
        UNRETURNING = 0x00,
        RETURNING
    }Stepper_Return_Status;

    typedef enum
    {
        RETURN_ERROR = 0x01,
        FORWARD_LIMIT = 0x06,
        REVERSE_LIMIT = 0x07,
    }Stepper_Limit_Status;

    typedef struct motor_status_str
    {
        Stepper_Running_Status motor_running_status;
        Stepper_Return_Status return_status;
        Stepper_Limit_Status limit_status;
    }Stepper_motor_Status;

    Stepper_motor_Status outin_motor, lr_motor, scrap_motor;

    void can_init();
    int canbus_send(unsigned int id, unsigned char *data, unsigned int len);
    int canbus_read(void);
    void elmo_enable(int elmo_id);
    void elmo_setmod(int elmo_id,uint8_t mod);
    void elmo_setVelocity(int elmo_id,int32_t Velocity);
    void elmo_positionVelocity(int elmo_id,int32_t Velocity);  //位置模式速度
    void elmo_setabsposition(int elmo_id,int32_t position);     //绝对位置
    void elmo_setrelposition(int elmo_id,int32_t position);     //相对位置
    void elmo_Disable(int elmo_id);

    void Stepper_setVelocity(int id,float_t Velocity);
    void Stepper_run_withVelocity(int id, int32_t direction, float_t Velocity);
    void Stepper_stop(int id);
    void Stepper_run_withabsPosition(int id, int32_t position, float_t Velocity);
    void Stepper_run_withrelPosition(int id, int32_t position, char direction, float_t Velocity);
    void Stepper_findzeroPosition(int id);
    void Stepper_checkStatus(int id);

    int getMotorStatus(int id);

    void Gripper_clamp(void);
    void Gripper_loosen(void);
};

Me_arm::Me_arm(/* args */)
{
    rfilter.can_id = 0x001;
    rfilter.can_mask = 0x7FF;
    outin_motor = {Stepper_Running_Status::FREE, Stepper_Return_Status::UNRETURNING, Stepper_Limit_Status::RETURN_ERROR};
    can_init();
}

Me_arm::~Me_arm()
{
}

void Me_arm::can_init()
{
	struct sockaddr_can addr;
	struct ifreq ifr;	

	//创建socket
	if((fd = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
		perror("Error while creating CAN:socket");
		return ;
	}
	
	strcpy(ifr.ifr_name, "can1");      //波特率为125k
    if (ioctl(fd, SIOCGIFINDEX, &ifr) < 0) {
        perror("ioctl failed");
        return ;
    }
	
	addr.can_family  = PF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;

	setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter)); //设置过滤器 只允许接受步进电机的数据，忽略elmo

	if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) { //绑定套接字 就是 套接字和canbus外设进行绑定
		perror("bind failed!");
		return ;
	}
    printf("Can1 init successfully!\r\n"); 
}

int Me_arm::canbus_send(unsigned int id, unsigned char *data, unsigned int len)
{
	int i,ret;

	struct can_frame frame;

	if((id > 0x7ff) || (len > 8)){
		perror("canbus id or data len error!");
        return -1;
	}

	frame.can_id = (canid_t)id;
	frame.can_dlc = len;
    // if(id == BIG_LR || id == SCRAP)
	//     printf("[%x]:", id);
	for(i = 0; i < len; i++){
		frame.data[i] = data[i];
        // if(id == BIG_LR || id == SCRAP)
        //     printf("%x ",data[i]);
	}
    // if(id == BIG_LR || id == SCRAP)
    //     printf("\n");
	ret = write(fd,&frame,sizeof(frame));
    usleep(2000);  //2ms
	if(ret < 0){
		perror("data write error!");
	}

	return ret;

}

int Me_arm::canbus_read(void)
{
	int len,i;
	struct can_frame buff;	
	// printf("can receive data!\n");
	len = read(fd,&buff,sizeof(buff));
	if( len == sizeof(buff))
	{
		rxmsg.CANTxStruct.StdId = buff.can_id;
		rxmsg.CANTxStruct.DLC = buff.can_dlc;
		for( i = 0 ; i < buff.can_dlc ; i++)
		{
			rxmsg.CANTxStruct.Data[i] = buff.data[i];
			// printf("%x ",buff.data[i]);
			
		}
		// printf("\n");
		// printf("id:%x,len:%x\n",rxmsg.CANTxStruct.StdId,rxmsg.CANTxStruct.DLC);
	}else{
		printf("CAN receive error!!!\n");
		return -1; //接收错误
	}

	return 0;
}

void Me_arm::elmo_enable(int elmo_id)
{
    unsigned char data[8];
    data[0] = 0x22;
    data[1] = 0x40;
    data[2] = 0x60;
    data[3] = 0x00;
    data[4] = 0x06;
    data[5] = 0x00;
    data[6] = 0x00;
    data[7] = 0x00;
    canbus_send(0x600+elmo_id,data,8);

    data[0] = 0x22;
    data[1] = 0x40;
    data[2] = 0x60;
    data[3] = 0x00;
    data[4] = 0x07;
    data[5] = 0x00;
    data[6] = 0x00;
    data[7] = 0x00;
    canbus_send(0x600+elmo_id,data,8);

    data[0] = 0x22;
    data[1] = 0x40;
    data[2] = 0x60;
    data[3] = 0x00;
    data[4] = 0x0f;
    data[5] = 0x00;
    data[6] = 0x00;
    data[7] = 0x00;
    canbus_send(0x600+elmo_id,data,8);
}

void Me_arm::elmo_setmod(int elmo_id,uint8_t mod)
{
    unsigned char data[8];
    data[0] = 0x22;
    data[1] = 0x60;
    data[2] = 0x60;
    data[3] = 0x00;
    data[4] = mod;
    data[5] = 0x00;
    data[6] = 0x00;
    data[7] = 0x00;
    canbus_send(0x600+elmo_id,data,8);
}

void Me_arm::elmo_setVelocity(int elmo_id, int32_t Velocity)    //速度模式设置速递后开始动
{
    DATA32_t candata;
    candata.data32 = Velocity;
    unsigned char data[8];
    data[0] = 0x22;
    data[1] = 0xff;
    data[2] = 0x60;
    data[3] = 0x00;
    data[4] = candata.raw[0];
    data[5] = candata.raw[1];
    data[6] = candata.raw[2];
    data[7] = candata.raw[3];
    canbus_send(0x600+elmo_id,data,8);
}

void Me_arm::elmo_positionVelocity(int elmo_id,int32_t Velocity)
{
    DATA32_t candata;
    candata.data32 = Velocity;
    unsigned char data[8];
    data[0] = 0x22;
    data[1] = 0x81;
    data[2] = 0x60;
    data[3] = 0x00;
    data[4] = candata.raw[0];
    data[5] = candata.raw[1];
    data[6] = candata.raw[2];
    data[7] = candata.raw[3];
    canbus_send(0x600+elmo_id,data,8);
}

void Me_arm::elmo_setabsposition(int elmo_id,int32_t position)
{
    DATA32_t candata;
    candata.data32 = position;
    unsigned char data[8];
    data[0] = 0x22;
    data[1] = 0x7a;
    data[2] = 0x60;
    data[3] = 0x00;
    data[4] = candata.raw[0];
    data[5] = candata.raw[1];
    data[6] = candata.raw[2];
    data[7] = candata.raw[3];
    canbus_send(0x600+elmo_id,data,8);

    data[0] = 0x22;
    data[1] = 0x40;
    data[2] = 0x60;
    data[3] = 0x00;
    data[4] = 0x1f;
    data[5] = 0x00;
    data[6] = 0x00;
    data[7] = 0x00;
    canbus_send(0x600+elmo_id,data,8);

    data[0] = 0x22;
    data[1] = 0x40;
    data[2] = 0x60;
    data[3] = 0x00;
    data[4] = 0x0f;
    data[5] = 0x00;
    data[6] = 0x00;
    data[7] = 0x00;
    canbus_send(0x600+elmo_id,data,8);
}

void Me_arm::elmo_setrelposition(int elmo_id,int32_t position)
{
    DATA32_t candata;
    candata.data32 = position;
    unsigned char data[8];
    data[0] = 0x22;
    data[1] = 0x7a;
    data[2] = 0x60;
    data[3] = 0x00;
    data[4] = candata.raw[0];
    data[5] = candata.raw[1];
    data[6] = candata.raw[2];
    data[7] = candata.raw[3];
    canbus_send(0x600+elmo_id,data,8);

    data[0] = 0x22;
    data[1] = 0x40;
    data[2] = 0x60;
    data[3] = 0x00;
    data[4] = 0x5f;
    data[5] = 0x00;
    data[6] = 0x00;
    data[7] = 0x00;
    canbus_send(0x600+elmo_id,data,8);

    data[0] = 0x22;
    data[1] = 0x40;
    data[2] = 0x60;
    data[3] = 0x00;
    data[4] = 0x0f;
    data[5] = 0x00;
    data[6] = 0x00;
    data[7] = 0x00;
    canbus_send(0x600+elmo_id,data,8);
}

void Me_arm::elmo_Disable(int elmo_id)
{
    unsigned char data[8];
    data[0] = 0x22;
    data[1] = 0x40;
    data[2] = 0x60;
    data[3] = 0x00;
    data[4] = 0x00;
    data[5] = 0x00;
    data[6] = 0x00;
    data[7] = 0x00;
    canbus_send(0x600+elmo_id,data,8);
}

void Me_arm::Stepper_setVelocity(int id,float_t Velocity)
{
    unsigned char data[8];
    FLOAT32_t candata;
    candata.data32 = Velocity;
    data[0] = 0x00;
    data[1] = 0x20;
    data[2] = (0x01<<5) | 0x06;

    data[3] = candata.raw[0];
    data[4] = candata.raw[1];
    data[5] = candata.raw[2];
    data[6] = candata.raw[3];
    data[7] = 0x00;
    canbus_send(id,data,8);
}

void Me_arm::Stepper_run_withVelocity(int id, int32_t direction, float_t Velocity)
{
    unsigned char data[8];
    this->Stepper_setVelocity(id, Velocity);
    data[0] = 0x00;
    data[1] = 0x20;
    if(direction >0)
        data[2] = (0x01<<5) | 0x03;  //正转
    else
        data[2] = (0x01<<5) | 0x04;   //反转

    data[3] = 0;
    data[4] = 0;
    data[5] = 0;
    data[6] = 0;
    data[7] = 0x03;
    canbus_send(id,data,8);
}

void Me_arm::Stepper_run_withrelPosition(int id, int32_t position, char direction, float_t Velocity)
{
    unsigned char data[8];
    this->Stepper_setVelocity(id, Velocity);
    DATA32_t candata;
    candata.data32 = position;
    data[0] = 0x00;
    data[1] = 0x20;
    if(direction >0)
        data[2] = (0x01<<5) | 0x03;  //正转
    else
        data[2] = (0x01<<5) | 0x04;   //反转

    data[3] = candata.raw[0];
    data[4] = candata.raw[1];
    data[5] = candata.raw[2];
    data[6] = candata.raw[3];
    // data[3] = 0;
    // data[4] = 0;
    // data[5] = 0;
    // data[6] = 0;
    data[7] = 0x03;
    canbus_send(id,data,8);
}

void Me_arm::Stepper_run_withabsPosition(int id, int32_t position, float_t Velocity)
{
    unsigned char data[8];
    this->Stepper_setVelocity(id, Velocity);
    DATA32_t candata;
    candata.data32 = position;
    data[0] = 0x00;
    data[1] = 0x20;
    data[2] = (0x01<<5) | 0x02;  
    data[3] = candata.raw[0];
    data[4] = candata.raw[1];
    data[5] = candata.raw[2];
    data[6] = candata.raw[3];
    data[7] = 0x01;
    canbus_send(id,data,8);
}

void Me_arm::Stepper_stop(int id)
{
    unsigned char data[8];
    data[0] = 0x00;
    data[1] = 0x20;
    data[2] = (0x01<<5) | 0x05;   //反转
    data[3] = 0;
    data[4] = 0;
    data[5] = 0;
    data[6] = 0;
    data[7] = 0x02;
    canbus_send(id,data,8);
}

void Me_arm::Stepper_findzeroPosition(int id)
{
    unsigned char data[8];
    data[0] = 0x00;
    data[1] = 0x20;
    data[2] = (0x01<<5) | 0x01;   //反转
    data[3] = 0;
    data[4] = 0;
    data[5] = 0;
    data[6] = 0;
    data[7] = 0x02;
    canbus_send(id,data,8);
}

void Me_arm::Stepper_checkStatus(int id)
{
    unsigned char data[8];
    data[0] = 0x00;
    data[1] = 0x20;
    data[2] = (0x01<<5) | 0x00;   
    data[3] = 0;
    data[4] = 0;
    data[5] = 0;
    data[6] = 0;
    data[7] = 0x02;
    canbus_send(id, data, 8);
    usleep(200);
    int ret = canbus_read();
    if(!ret)    //有数据
    {
        uint8_t ByteData = rxmsg.CANTxStruct.Data[7];
        if(ByteData != 0xFF)
        {
            if(id == OUTIN)
            {
                this->outin_motor.motor_running_status = (Stepper_Running_Status)(ByteData & 0x07);
                this->outin_motor.return_status = (Stepper_Return_Status)(( ByteData & 0x08 ) >> 3);
                this->outin_motor.limit_status = (Stepper_Limit_Status)(( ByteData & 0xF0 ) >> 4);
            }
            else if (id == BIG_LR)
            {
                this->lr_motor.motor_running_status = (Stepper_Running_Status)(ByteData & 0x07);
                this->lr_motor.return_status = (Stepper_Return_Status)(( ByteData & 0x08 ) >> 3);
                this->lr_motor.limit_status = (Stepper_Limit_Status)(( ByteData & 0xF0 ) >> 4);            
            }
        }
    }
    else
        return;
}

int Me_arm::getMotorStatus(int id)
{
    if(id == OUTIN)
        return (int)(this->outin_motor.motor_running_status);
    else if(id == BIG_LR)
        return (int)(this->lr_motor.motor_running_status);
    else if(id == SCRAP)
        return (int)(this->scrap_motor.motor_running_status);
    else
        return -1;
}

void Me_arm::Gripper_clamp(void)
{
    this->Stepper_run_withVelocity(SCRAP, 1, 200);
    sleep(4);
    // usleep(500000);
    this->Stepper_stop(SCRAP);
}

void Me_arm::Gripper_loosen(void)
{
    this->Stepper_run_withVelocity(SCRAP, -1, 200);
    sleep(4);
    // usleep(500000);
    this->Stepper_stop(SCRAP);
}


