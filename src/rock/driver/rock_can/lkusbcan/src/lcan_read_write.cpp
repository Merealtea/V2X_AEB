#include "lcan_common.h"

using namespace std;
#define __countof(a) (sizeof(a) / sizeof(a[0]))

#define SEND_FRAMES 1  // 每次发送帧数
#define SEND_TIMES 1   // 发送次数

#define USE_CAN_NUM 1  // 1-CAN0自发自收；2-CAN0、CAN1对发

DWORD dwDeviceHandle_0;
DWORD dwDeviceHandle_1;
char count1;
char count2;

bool canid_0x111_updated;
bool canid_0x112_updated;

CAN_DataFrame cansend_0x111;
CAN_DataFrame cansend_0x112;

CAN_DataFrame can_send[2];

typedef struct {
  int Run;
  DWORD ch;
} thread_arg_t;
int CanSendcount[2] = {0, 0};

ros::Subscriber sub;
ros::Subscriber sub1;

ros::Subscriber sub2;
ros::Subscriber sub3;

ros::Publisher canframe_pub;
ros::Publisher srrframe_pub;
int write_flag = 0;
cyber_msgs::canframe ControlCmd;

int can0_flag;
int can1_flag;

void *receive_func_0(void *param)  //接收线程的处理函数
{
  int reclen = 0;
  thread_arg_t *thread_arg = (thread_arg_t *)param;
  int ind = 0;
  int count = 0;
  int errcount = 0;
  CAN_DataFrame rec[100];
  int i;
  CAN_DataFrame snd;
  CAN_ErrorInformation err;
  printf("receive thread0 running....\n");
  while (thread_arg->Run) {
    if ((reclen = CAN_ChannelReceive(dwDeviceHandle_0, ind, rec, __countof(rec),
                                     10)) > 0) {
      //         printf("CAN%d Receive: %08X", ind, rec[reclen - 1].uID);
      //         printf("\treclen%d\t",reclen);
      //--------------------------------------------------
      for (int num = 0; num < reclen; num++) {
        if (rec[num].uID == 0x2a3 || rec[num].uID == 0x2a4 ||
            rec[num].uID == 0x21b || rec[num].uID == 0x25b ||
            rec[num].uID == 0x27a || rec[num].uID == 0x108)  // vehicle
        {
          cyber_msgs::canframe rec_frame;
          rec_frame.id = rec[num].uID;
          rec_frame.len = rec[num].nDataLen;
          rec_frame.header.stamp = ros::Time::now();

          for (int t = 0; t < 8; t++)  // 8位
            rec_frame.data[t] = 0;
          for (int t = 0; t < rec_frame.len; t++)
            rec_frame.data[t] = (int)rec[num].arryData[t];
          canframe_pub.publish(rec_frame);
          usleep(200);
        }
        // printf("0:%d\n",can0_flag);
      }
      //---------------------------------------------------
      //          for ( i = 0; i < rec[reclen - 1].nDataLen; i++ )
      //          {
      //              printf(" %02X", rec[reclen - 1].arryData[i]);
      //          }
      //          printf("\n");
      count += reclen;
      //          printf("CAN%d rcv count=%d\n", ind, count);

    } else {
      if (CAN_GetErrorInfo(dwDeviceHandle_0, ind, &err) == CAN_RESULT_OK) {
        errcount++;
      }
      usleep(200);
    }
  }
  printf("CAN%d rcv count=%d err count:%d\n", 0, count, errcount);
}

void *send_func_0(void *param) {
  int ch = 0;
  int i;
  int count = 0;
  cout << "进入发送线程0..." << endl;

  while (1) {
    count = 0;
    if (canid_0x111_updated == 1) {
      can_send[count] = cansend_0x111;
      count++;
      canid_0x111_updated = 0;
    }
    if (canid_0x112_updated == 1) {
      can_send[count] = cansend_0x112;
      count++;
      canid_0x112_updated = 0;
    }

    std::cout << "A" << count << endl;

    if (count > 0) {
      // for ( i = 0; i < count; i++ )
      //              printf("CAN%d Send %d\r\n", ch, times);
      unsigned long sndCnt =
          CAN_ChannelSend(dwDeviceHandle_0, ch, can_send, count);
      std::cout << "B" << sndCnt << endl;
    }
    usleep(5000);
  }
  // usleep(10000);
}

void ctl111MsgCallback(const cyber_msgs::canframe &can_msg) {
  canid_0x111_updated = 1;
  cansend_0x111.uID = can_msg.id;
  cansend_0x111.nSendType =
      1;  // 0-正常发送;1-单次发送;2-自发自收;3-单次自发自收
  cansend_0x111.bRemoteFlag = 0;         // 0-数据帧；1-远程帧
  cansend_0x111.bExternFlag = 0;         // 0-标准帧；1-扩展帧
  cansend_0x111.nDataLen = can_msg.len;  // DLC
  for (int i = 0; i < 8; i++) {
    cansend_0x111.arryData[i] = (char)can_msg.data[i];
  }
}

void ctl112MsgCallback(const cyber_msgs::canframe &can_msg) {
  canid_0x112_updated = 1;
  cansend_0x112.uID = can_msg.id;
  cansend_0x112.nSendType =
      1;  // 0-正常发送;1-单次发送;2-自发自收;3-单次自发自收
  cansend_0x112.bRemoteFlag = 0;         // 0-数据帧；1-远程帧
  cansend_0x112.bExternFlag = 0;         // 0-标准帧；1-扩展帧
  cansend_0x112.nDataLen = can_msg.len;  // DLC
  for (int i = 0; i < 8; i++) {
    cansend_0x112.arryData[i] = (char)can_msg.data[i];
  }
}

void *receive_func_1(void *param)  //接收线程的处理函数
{
  int reclen = 0;
  thread_arg_t *thread_arg = (thread_arg_t *)param;
  int ind = 0;
  int count = 0;
  int errcount = 0;
  CAN_DataFrame rec[100];
  int i;
  CAN_DataFrame snd;
  CAN_ErrorInformation err;
  printf("receive thread1 running....\n");
  while (thread_arg->Run) {
    if ((reclen = CAN_ChannelReceive(dwDeviceHandle_1, ind, rec, __countof(rec),
                                     10)) > 0) {
      //         printf("CAN%d Receive: %08X", ind, rec[reclen - 1].uID);
      //         printf("\treclen%d\t",reclen);
      //--------------------------------------------------
      for (int num = 0; num < reclen; num++) {
        if (rec[num].uID == 0x301)
          can1_flag = 0;  // GL8's CAN
        else if (rec[num].uID == 0x4E0)
          can1_flag = 1;  // SRR's CAN
        if (can1_flag == -1)
          continue;
        else if (can1_flag == 0) {
          if ((rec[num].uID == 0x301 || rec[num].uID == 0x302 ||
               rec[num].uID == 0x303)  // vehicle feedbacks
              || (rec[num].uID == 0x6A7 ||
                  rec[num].uID == 0x6A6)  // mobileye lane line
              || (rec[num].uID == 0x740 || rec[num].uID == 0x741 ||
                  rec[num].uID == 0x760 ||
                  rec[num].uID == 0x761))  // mobileye objects
          {
            cyber_msgs::canframe rec_frame;
            rec_frame.id = rec[num].uID;
            rec_frame.len = rec[num].nDataLen;
            rec_frame.header.stamp = ros::Time::now();

            for (int t = 0; t < 8; t++)  // 8位
              rec_frame.data[t] = 0;
            for (int t = 0; t < rec_frame.len; t++)
              rec_frame.data[t] = (int)rec[num].arryData[t];

            canframe_pub.publish(rec_frame);
            usleep(200);
          }
        } else if (can1_flag == 1) {
          cyber_msgs::canframe rec_frame;
          rec_frame.id = rec[num].uID;
          rec_frame.len = rec[num].nDataLen;
          rec_frame.header.stamp = ros::Time::now();

          for (int t = 0; t < 8; t++)  // 8位
            rec_frame.data[t] = 0;
          for (int t = 0; t < rec_frame.len; t++)
            rec_frame.data[t] = (int)rec[num].arryData[t];

          srrframe_pub.publish(rec_frame);
          usleep(200);
        }
        // printf("1:%d\n",can1_flag);
      }
      //---------------------------------------------------
      //          for ( i = 0; i < rec[reclen - 1].nDataLen; i++ )
      //          {
      //              printf(" %02X", rec[reclen - 1].arryData[i]);
      //          }
      //          printf("\n");
      count += reclen;
      //          printf("CAN%d rcv count=%d\n", ind, count);

    } else {
      if (CAN_GetErrorInfo(dwDeviceHandle_1, ind, &err) == CAN_RESULT_OK) {
        errcount++;
      }
      usleep(200);
    }
  }
  printf("CAN%d rcv count=%d err count:%d\n", 1, count, errcount);
}

void *send_func_1(void *param) {
  int ch = 0;
  int i;
  CAN_DataFrame send[SEND_FRAMES];
  cout << "进入发送线程1..." << endl;

  while (1) {
    //收到控制指令
    if (write_flag == 1) {
      int times = 1;  //  26000-12h
      send[0].uID = ControlCmd.id;
      send[0].nSendType = 0;  // 0-正常发送;1-单次发送;2-自发自收;3-单次自发自收
      send[0].bRemoteFlag = 0;            // 0-数据帧；1-远程帧
      send[0].bExternFlag = 0;            // 0-标准帧；1-扩展帧
      send[0].nDataLen = ControlCmd.len;  // DLC
      for (i = 0; i < ControlCmd.len; i++)
        send[0].arryData[i] = (char)ControlCmd.data[i];

      //              printf("CAN%d Send %d\r\n", ch, times);
      unsigned long sndCnt = CAN_ChannelSend(dwDeviceHandle_0, ch, send, 1);
      CanSendcount[ch] += sndCnt;
      // if(sndCnt) times--;

      write_flag = 0;
      //            printf("CAN%d Send Count:%d end \r\n", ch,
      //            CanSendcount[ch]);
    }

    // usleep(10000);///zhangxiao delete
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "lcan_read_write");
  ros::NodeHandle nh;
  canframe_pub = nh.advertise<cyber_msgs::canframe>("/can_frame", 1);
  // srrframe_pub = nh.advertise<cyber_msgs::canframe>("radar_frame", 500);
  sub = nh.subscribe("/rock_can/can_write_0x111", 1, ctl111MsgCallback);
  sub1 = nh.subscribe("/rock_can/can_write_0x112", 1, ctl112MsgCallback);
  // sub2 = nh.subscribe("/rock_can/can_write_0x131_brake", 1,
  // ctl131MsgCallback); sub3 = nh.subscribe("/rock_can/can_write_0x632_enable",
  // 1, ctl632MsgCallback);

  can0_flag = 0;
  can1_flag = -1;
  count1 = 0;
  count2 = 0;

  //  Status = CAN_Initialize(PCAN_USBBUS1, PCAN_BAUD_250K, 0, 0, 0);
  //  printf("Initialize CAN: %i\n", (int) Status);

  CAN_InitConfig config;
  int i = 0;
  thread_arg_t rcv_thread_arg0;
  thread_arg_t rcv_thread_arg1;
  pthread_t rcv_threadid0;
  pthread_t rcv_threadid1;
  pthread_t snd_threadid0;
  pthread_t snd_threadid1;
  int ret;
  char c;
  int times = 100;
  int sendind = 3;
  time_t tm1, tm2;
  int minute, second;
  while ((dwDeviceHandle_0 = CAN_DeviceOpen(ACUSB_132B, 0, 0)) == 0) {
    printf("laikeusbcan open deivce0 error0\n");
    usleep(1000 * 1000);
  }
  /*if ( (dwDeviceHandle_1 = CAN_DeviceOpen(ACUSB_132B, 1, 0)) == 0 ) {
      printf("open deivce1 error1\n");
  }*/
  config.dwAccCode = 0;
  config.dwAccMask = 0xffffffff;
  config.nFilter = 0;  // 滤波方式(0表示未设置滤波功能,1表示双滤波,2表示单滤波)
  config.bMode = 0;     // 工作模式(0表示正常模式,1表示只听模式)
  config.nBtrType = 1;  // 位定时参数模式(1表示SJA1000,0表示LPC21XX)
  config.dwBtr[0] =
      0x00;  // BTR0   0014 -1M 0016-800K 001C-500K 011C-250K 031C-12K 041C-100K
             // 091C-50K 181C-20K 311C-10K BFFF-5K
  config.dwBtr[1] = 0x1C;  // BTR1
  config.dwBtr[2] = 0;
  config.dwBtr[3] = 0;
  if (CAN_ChannelStart(dwDeviceHandle_0, 0, &config) != CAN_RESULT_OK) {
    printf("laikeusbcan0 Start CAN 0 error\n");
  }
#if USE_CAN_NUM > 1
  if (CAN_ChannelStart(dwDeviceHandle_0, 1, &config) != CAN_RESULT_OK) {
    printf("laikeusbcan0 Start CAN 1 error\n");
  }
#endif
  rcv_thread_arg0.Run = TRUE;
  rcv_thread_arg0.ch = 0;
  rcv_thread_arg1.Run = TRUE;
  rcv_thread_arg1.ch = 1;
  ret = pthread_create(&rcv_threadid0, NULL, receive_func_0, &rcv_thread_arg0);
#if USE_CAN_NUM > 1
  ret = pthread_create(&rcv_threadid1, NULL, receive_func_1, &rcv_thread_arg1);
#endif
  ret = pthread_create(&snd_threadid0, NULL, send_func_0, (void *)0);
#if USE_CAN_NUM > 1
  ret = pthread_create(&snd_threadid1, NULL, send_func_1, (void *)1);
#endif
  //   time(&tm1);
  //   printf("create thread\n");
  //   pthread_join(snd_threadid0, NULL);

  ros::spin();

  time(&tm2);
  minute = (tm2 - tm1) / 60;
  second = (tm2 - tm1) % 60;
  printf(
      "CAN0 Sendcount:%d CAN1 Sendcount:%d With time: %d minute,%d second\r\n",
      CanSendcount[0], CanSendcount[1], minute, second);
  usleep(200000);  // 等待接收线程读取完所有帧
  // 退出接收线程
  rcv_thread_arg0.Run = FALSE;
  pthread_join(rcv_threadid0, NULL);
  rcv_thread_arg1.Run = FALSE;

  CAN_ChannelStop(dwDeviceHandle_0, 0);
  CAN_DeviceClose(dwDeviceHandle_0);
  CAN_ChannelStop(dwDeviceHandle_1, 0);
  CAN_DeviceClose(dwDeviceHandle_1);
  printf("CAN_DeviceClose\r\n");
  printf("Press Enter key to exit the program\r\n");
  scanf("%c", &c);
  //    return 0;
}
