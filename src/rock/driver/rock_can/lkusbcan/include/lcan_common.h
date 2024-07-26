#ifndef _LCAN_COMMON_H
#define _LCAN_COMMON_H

#include <iostream>
#include <string>

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Int8MultiArray.h"
#include "std_msgs/String.h"

// can frame
#include <cyber_msgs/canframe.h>

//----------与can驱动相关的头文件及宏定义--------------------
#include <fcntl.h>
#include <pthread.h>
#include <stdio.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include "ICANCmd.h"

//-----------------------------------------------------------

using namespace std;

#endif
