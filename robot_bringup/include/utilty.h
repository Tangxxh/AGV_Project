/*
 * @Author: Wei Zhifei
 * @Date: 2022-11-12 13:18:38
 */
#ifndef UTILTY_H
#define UTILTY_H

typedef signed char int8_t;
typedef signed short int int16_t;
typedef signed int int32_t;

typedef unsigned char uint8_t;
typedef unsigned short int uint16_t;
typedef unsigned int uint32_t;

#define CLASSIS_CMD_LENGTH     8       //命令长度

typedef enum{
	UART_HEAD1 = 0x00,
	UART_HEAD2 = 0x01,
	UART_OPERATE = 0x02,
	UART_TYPE = 0x03, 
	UART_DATA = 0x04,  
	UART_CHACK = 0x05,
}ENUM_UART_STATE;

typedef union
{
	int8_t raw[2];
	int16_t data16;
} DATA16_t;

typedef union
{
	int8_t raw[4];
	int32_t data32;
} DATA32_t;

/*底盘控制结构体*/
typedef struct chassiscmd
{
	uint8_t Node_ID;
	uint8_t CMD;
	DATA16_t data1;
	DATA16_t data2;
	uint8_t Byte6;
	uint8_t Byte7;
} ClassisCmdStruct;

/*底盘状态自动反馈信息 50hz*/
typedef struct chassisinfo1
{
	uint8_t Node_ID;
	uint8_t CMD;
	DATA16_t left_front_vel;
	DATA16_t right_front_vel;
	uint16_t voltage;
}ClassisVolStruct;

typedef struct chassisinfo2
{
	uint8_t Node_ID;
	uint8_t CMD;
	DATA16_t left_back_vel;
	DATA16_t right_back_vel;
	uint8_t Byte6;
	uint8_t Byte7;
} ClassisStateStruct;

typedef struct chassisinfo3
{
	uint8_t Node_ID;
	uint8_t CMD;
	DATA16_t left_front_encoder;
	DATA16_t right_front_encoder;
	uint8_t Byte6;
	uint8_t Byte7;
} ClassisFrontEncoderStruct;

typedef struct chassisinfo4
{
	uint8_t Node_ID;
	uint8_t CMD;
	DATA16_t left_back_encoder;
	DATA16_t right_back_encoder;
	uint8_t Byte6;
	uint8_t Byte7;
} ClassisBackEncoderStruct;



typedef union
{
	uint8_t cmddata[CLASSIS_CMD_LENGTH];
	ClassisCmdStruct cmdstruct;
	ClassisVolStruct cmdvolstruct;
	ClassisStateStruct cmdstatestruct;
	ClassisFrontEncoderStruct cmdfrontencoderstruct;
	ClassisBackEncoderStruct cmdbackencoderstruct;
}ClassisRecvCmd_t;


/*Can总线有关结构体*/
#define CAN_CMD_LENGTH 		28		//4*5 + 8(0~8) 

// CAN sendcmd struct
typedef union
{
	uint8_t	CANTxMsg[CAN_CMD_LENGTH];
	struct
	{
		uint32_t  StdId;
		uint32_t	ExtId;
		uint32_t	IDE;
		uint32_t	RTR;
		uint32_t	DLC;
		uint8_t		Data[8];
	}CANTxStruct;
}CANTxMsg_t;

/*激光测距有关结构体*/
typedef union
{
	uint8_t TOF_TX_Msg[6];
	struct
	{
		uint8_t		STX;
		uint8_t 	command;
		DATA16_t 	data;
		uint8_t		ETX;
		uint8_t		BCC;
	}TOF_TX_STR;
}TOF_Msg_t;

//KEYA 驱动器使能控制
typedef enum
{
	KEYADISABLE = 0x00,	//去使能--停止运动
	KEYAENABLE = 0x01,	//使能

}KEYAACTIVE_t;

#endif
