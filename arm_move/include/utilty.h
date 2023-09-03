#ifndef UTILTY_H
#define UTILTY_H

typedef signed char int8_t;
typedef signed short int int16_t;
typedef signed int int32_t;

typedef unsigned char uint8_t;
typedef unsigned short int uint16_t;
typedef unsigned int uint32_t;

#define CLASSIS_CMD_LENGTH     8       //命令长度

typedef union
{
  int8_t raw[2];
  int16_t data16;
} DATA16_t;

/* data union data32*/
typedef union
{
	uint8_t	raw[4];
	uint32_t data32;
}DATA32_t;

typedef struct urc
{
  uint8_t Node_ID;
  uint8_t CMD;
  DATA16_t data1;
  DATA16_t data2;
  uint8_t Byte6;
  uint8_t Byte7;
} ClassisCmdStruct;

typedef struct drc
{
  uint8_t Node_ID;
  uint8_t CMD;
  DATA16_t data1;
  DATA16_t data2;
  uint16_t voltage;
} ClassisVolStruct;

typedef union
{
  uint8_t cmddata[CLASSIS_CMD_LENGTH];
  ClassisCmdStruct cmdstruct;
  ClassisVolStruct cmdvolstruct;
} ClassisRecvCmd_t;

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

#endif
