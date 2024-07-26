#include <ros/ros.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <pthread.h>
#include <ctime>
#include <cstdlib>
#include <unistd.h>

#include <controlcan.h>
#include <cyber_msgs/canframe.h>


VCI_BOARD_INFO pInfo; //用来获取设备信息。
int count = 0;        //数据列表中，用来存储列表序号。
VCI_BOARD_INFO pInfo1[50];
int num = 0;

ros::Publisher canframe_pub;

void *receive_func(void *param) //接收线程
{
    int reclen = 0;
    VCI_CAN_OBJ rec[3000]; //接收缓存，设为3000为佳。
    int i, j;

    int *run = (int *)param; //线程启动，退出控制。
    int ind = 0;
    while ((*run) & 0x0f)
    {
        if ((reclen = VCI_Receive(VCI_USBCAN2, 0, ind, rec, 3000, 100)) > 0) //调用接收函数，如果有数据，进行数据处理显示。
        {
            for (j = 0; j < reclen; j++)
            {
                cyber_msgs::canframe canframe;
                canframe.header.stamp = ros::Time::now();
                canframe.id = rec[j].ID;
                canframe.len = rec[j].DataLen;
                for (int k = 0; k < 8; k++)
                    canframe.data[k] = 0;
                for (int k = 0; k < canframe.len; k++)
                    canframe.data[k] = rec[j].Data[k];
                printf("pub\n");
                canframe_pub.publish(canframe);
            }
        }
    }
    printf("run thread exit\n"); //退出接收线程
    pthread_exit(0);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "radar_can_driver");
    ros::NodeHandle nh;
    canframe_pub = nh.advertise<cyber_msgs::canframe>("/radar_can_frame", 1);

    // open CAN
    if(VCI_OpenDevice(VCI_USBCAN2,0,0)==1)//打开设备
    {
        printf(">>open deivce success!\n");//打开设备成功
    }
    else
    {
        printf(">>open deivce error!\n");
        exit(1);
    }
    // init CAN
    //初始化参数，严格参数二次开发函数库说明书
    VCI_INIT_CONFIG config;
    config.AccCode = 0;
    config.AccMask = 0xFFFFFFFF;
    config.Filter = 1;//接收所有帧
    config.Timing0 = 0x00;
    config.Timing1 = 0x1C;
    config.Mode = 0;//正常模式
    // use channel 0 only
    if(VCI_InitCAN(VCI_USBCAN2, 0, 0, &config) != 1)
    {
        printf(">>Init CAN1 error\n");
        VCI_CloseDevice(VCI_USBCAN2, 0);
    }
    if(VCI_StartCAN(VCI_USBCAN2, 0, 0) != 1)
    {
        printf(">>Start CAN1 error\n");
        VCI_CloseDevice(VCI_USBCAN2, 0);
    }

    // // configure radar
    // VCI_CAN_OBJ send[1];
    // send[0].ID = 0x200;
    // send[0].SendType = 0;
    // send[0].RemoteFlag = 0;
    // send[0].ExternFlag = 1;
    // send[0].DataLen = 8;
    // send[0].Data[0] = 0xC8;
    // send[0].Data[1] = 0x00;
    // send[0].Data[2] = 0x00;
    // send[0].Data[3] = 0x00;
    // send[0].Data[4] = 0x18; // 0x00 -- return none, 0x08 -- return objects, 0x18 -- return clusters 
    // send[0].Data[5] = 0x90;
    // send[0].Data[6] = 0x00;
    // send[0].Data[7] = 0x00;
    // if (VCI_Transmit(VCI_USBCAN2, 0, 0, send, 1) == 1) printf(">>Configure Radar success!\n");
    // else
    // {
    //     printf(">>Configure Radar error!\n");
    //     VCI_CloseDevice(VCI_USBCAN2, 0);
    //     exit(1);
    // }
    

    int m_run0 = 1;
    pthread_t threadid;
    int ret;
    ret = pthread_create(&threadid, NULL, receive_func, &m_run0);

    ros::spin();

    usleep(10000000);//延时单位us，这里设置 10 000 000=10s    10s后关闭接收线程，并退出主程序。
    m_run0 = 0;//线程关闭指令。
    pthread_join(threadid,NULL);//等待线程关闭。
    usleep(100000);//延时100ms。
    VCI_ResetCAN(VCI_USBCAN2, 0, 0);//复位CAN1通道。
    usleep(100000);//延时100ms。
    // VCI_ResetCAN(VCI_USBCAN2, 0, 1);//复位CAN2通道。
    // usleep(100000);//延时100ms。
    VCI_CloseDevice(VCI_USBCAN2, 0);//关闭设备。
    //除收发函数外，其它的函数调用前后，最好加个毫秒级的延时，即不影响程序的运行，又可以让USBCAN设备有充分的时间处理指令。

    return 0;
}