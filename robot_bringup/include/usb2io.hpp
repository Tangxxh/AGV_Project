#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "usb2gpio.h"
#include "usb_device.h"

enum Pin_Mode
{
    OUT_GPIO_PUPD_NOPULL = 1,//输出——没上下拉
    OUT_GPIO_PUPD_UP     = 2,//输出——上拉
    OUT_GPIO_PUPD_DOWN   = 3,//输出——下拉

    IN_GPIO_PUPD_NOPULL  = 4,//输入——没上下拉
    IN_GPIO_PUPD_UP      = 5,//输入——上拉
    IN_GPIO_PUPD_DOWN    = 6,//输入——下拉

    OPENIN_GPIO_PUPD_NOPULL = 7,//开漏输入——没上下拉
    OPENIN_GPIO_PUPD_UP     = 8,//开漏输入——上拉
    OPENIN_GPIO_PUPD_DOWN   = 9 //开漏输入——下拉
};

class USB2GPIO
{
private:
   int DevHandle[1];
public:
    USB2GPIO();
    ~USB2GPIO();
    void GPIO_SetPinMode(int Pin,int Mode);
    void GPIO_WrirePin(int Pin,int PinValue);
    bool  GPIO_Readepin(int Pin);
};

USB2GPIO::USB2GPIO()
{
    if(USB_ScanDevice(DevHandle) > 0) 
    {
        printf("Found USB-IO Devices:");
        printf("%08X \n",DevHandle[0]);
    }
    else{
        printf("No usbio device connected!\n");
    }

    if(USB_OpenDevice(DevHandle[0]) == 0)
    {
        printf("Open usbio device error!\n");
    }
    for(int i=0;i<8;i++)
    {
        GPIO_SetPinMode(i,OUT_GPIO_PUPD_DOWN);
        GPIO_WrirePin(i,0);
    }
    for(int j=8;j<16;j++)
    {
        GPIO_SetPinMode(j,IN_GPIO_PUPD_UP);
        GPIO_Readepin(j);  
    }
}
USB2GPIO::~USB2GPIO()
{
    for(int i=0;i<8;i++)
    {
        GPIO_SetPinMode(i,OUT_GPIO_PUPD_DOWN);
        GPIO_WrirePin(i,0);
    }
    for(int j=8;j<16;j++)
    {
        GPIO_SetPinMode(j,IN_GPIO_PUPD_UP);
    }
    USB_CloseDevice(DevHandle[0]);
}


void USB2GPIO::GPIO_SetPinMode(int Pin,int Mode)
{
    uint16_t Temp = 0;
    Temp = 1 << Pin;
    printf("Temp:%#x \n",Temp);
    switch (Mode)
    {
    case OUT_GPIO_PUPD_NOPULL:
        GPIO_SetOutput(DevHandle[0],Temp,GPIO_PUPD_NOPULL);
        break;
    case OUT_GPIO_PUPD_UP:
        GPIO_SetOutput(DevHandle[0],Temp,GPIO_PUPD_UP);
        break;
    case OUT_GPIO_PUPD_DOWN:
        GPIO_SetOutput(DevHandle[0],Temp,GPIO_PUPD_DOWN);
        break;
    case IN_GPIO_PUPD_NOPULL:
        GPIO_SetInput(DevHandle[0],Temp,GPIO_PUPD_NOPULL);
        break;
    case IN_GPIO_PUPD_UP:
        GPIO_SetInput(DevHandle[0],Temp,GPIO_PUPD_UP);
        break;
    case IN_GPIO_PUPD_DOWN:
        GPIO_SetInput(DevHandle[0],Temp,GPIO_PUPD_DOWN);
        break;
    case OPENIN_GPIO_PUPD_NOPULL:
        GPIO_SetOpenDrain(DevHandle[0],Temp,GPIO_PUPD_NOPULL);
        break;
    case OPENIN_GPIO_PUPD_UP:
        GPIO_SetOpenDrain(DevHandle[0],Temp,GPIO_PUPD_UP);
        break;
    case OPENIN_GPIO_PUPD_DOWN:
        GPIO_SetOpenDrain(DevHandle[0],Temp,GPIO_PUPD_DOWN);
        break;
    default:
        break;
    }
}

void USB2GPIO::GPIO_WrirePin(int Pin,int PinValue)
{
    uint16_t Temp;
    Temp = 1 << Pin;
    GPIO_Write(DevHandle[0],Temp,0xFFFF*PinValue);
}

bool USB2GPIO::GPIO_Readepin(int Pin)
{
    unsigned int PinValue;
    uint16_t Temp;
    Temp = 1 << Pin;
    GPIO_Read(DevHandle[0],Temp,&PinValue);
    if((PinValue&Temp)!=0x00){
        printf("P[%d]为高电平\n",Pin);
        return 1;
    }else{
        printf("P[%d]为低电平\n",Pin);
        return 0;
    }
}

    DEVICE_INFO DevInfo;
    
    bool state;
    int ret;
    unsigned int PinValue;
    int DevHandle[10];


