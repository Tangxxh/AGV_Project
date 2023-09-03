#include "HCNetSDK.h"
#include <cstring>
#include <iostream>
#include <unistd.h>


#include <ros/ros.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Bool.h>
#include <hk_sdk_arm/hk_control.h>

#pragma execution_character_set("utf-8")

LONG lUserID = 1;

void CALLBACK g_ExceptionCallBack(DWORD dwType, LONG lUserID, LONG lHandle, void *pUser)
{
    char tempbuf[256] = {0};
    switch(dwType)
    {
    case EXCEPTION_RECONNECT: //预览时重连
        break;
    default:
        break;

    }
}

using namespace std;

bool sethk_callback(hk_sdk_arm::hk_control::Request &req, hk_sdk_arm::hk_control::Response &res){
    res.ret = NET_DVR_PTZPreset_Other(lUserID,1,GOTO_PRESET,req.num);
    return true;
}

// void stateCallBack(const std_msgs::Int64::ConstPtr &msgs)
// {
//     if(msgs->data == 3)
//     {
//         systemState = 3;
//     }
// }


int main(int argc,char* argv[])
{
    LONG lRealPlayHandle=0;//设备句柄
    LONG nPort = -1;

    ros::init(argc,argv,"hk_node");
    //HWND hWnd = NULL;
    // 初始化
    NET_DVR_Init();
    //设置连接时间与重连时间
    NET_DVR_SetConnectTime(2000, 1);
    NET_DVR_SetReconnect(10000, true);

    //设置异常消息回调函数
    NET_DVR_SetExceptionCallBack_V30(0,NULL,g_ExceptionCallBack,NULL);

    //注册设备
    NET_DVR_USER_LOGIN_INFO struLoginInfo = {0};
    struLoginInfo.bUseAsynLogin = 0; //同步登录方式
    strcpy(struLoginInfo.sDeviceAddress, "192.168.1.64"); //设备IP地址
    struLoginInfo.wPort = 8000; //设备服务端口
    strcpy(struLoginInfo.sUserName, "admin"); //设备登录用户名
    strcpy(struLoginInfo.sPassword, "abcd1234"); //设备登录密码

    //设备信息, 输出参数
    NET_DVR_DEVICEINFO_V40 struDeviceInfoV40 = {0};
    lUserID = NET_DVR_Login_V40(&struLoginInfo, &struDeviceInfoV40);
    //hWnd = (HWND)ui->lb->winId();

    //启动预览
    // NET_DVR_PREVIEWINFO struPlayInfo={0};
    // struPlayInfo.hPlayWnd = hWnd;
    // struPlayInfo.lChannel = 1;                //预览通道
    // struPlayInfo.dwStreamType = 0;      //0主码流
    // struPlayInfo.dwLinkMode = 0;        //TCP方式
    // struPlayInfo.bBlocked = 0; //0- 非阻塞取流，1- 阻塞取流

    // lRealPlayHandle  = NET_DVR_RealPlay_V40(lUserID, &struPlayInfo, NULL, NULL);
	// NET_DVR_PTZControl_Other(lUserID,1,PAN_LEFT,0);
	// sleep(5);
	// NET_DVR_PTZControl_Other(lUserID,1,PAN_LEFT,1);
    ros::NodeHandle nh;
    ros::ServiceServer server = nh.advertiseService("hk_set_ori",sethk_callback);
    // ros::Subscriber subSystemState = nh.subscribe<std_msgs::Int64>("/system_state",1,stateCallBack);
    // ros::Publisher pubSystemState = nh.advertise<std_msgs::Int64>("/system_state",1);
    ros::spin();
}
