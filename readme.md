# SJTU Cyberc3 V2X AEB
***

- [SJTU Cyberc3 V2X AEB](#sjtu-cyberc3-v2x-aeb)
  - [硬件结构](#硬件结构)
    - [边缘云服务器](#边缘云服务器)
    - [长安rock](#长安rock)
    - [合创Hycan](#合创hycan)
  - [环境配置](#环境配置)
    - [中心云服务器](#中心云服务器)
    - [车端orin](#车端orin)
  - [启动流程](#启动流程)
    - [1. 启动车辆，布设边缘云服务器](#1-启动车辆布设边缘云服务器)
    - [2. 时间同步](#2-时间同步)
    - [3. 设置通讯IP](#3-设置通讯ip)
    - [4. 编译系统](#4-编译系统)
    - [5. 启动系统](#5-启动系统)
      - [边缘云](#边缘云)
      - [长安车](#长安车)
      - [合创车](#合创车)


## 硬件结构

### 边缘云服务器

边缘云服务器由一个路由器和一个笔记本/主机组成，负责接收车端发送的数据，进行融合后返回到车端

### 长安rock

长安rock车辆由一个orin、一个Xavier和右nuc组成，负责接收摄像头数据和GPS定位数据，进行目标检测后将检测框发送到边缘云服务器。之后再接收边缘云服务器返回的融合结果，进行AEB控制。

### 合创Hycan

合创Hycan车辆由一个orin和一个Xavier组成，负责接收摄像头数据和GPS定位数据，进行目标检测后将检测框发送到边缘云服务器。之后再接收边缘云服务器返回的融合结果，进行AEB控制。


## 环境配置
### 中心云服务器

TBD：等后面搞个docker，现在要求安装ros，python3（>=3.8），rospkg和chrony就可以

### 车端orin

TBD:等后面写个脚本来配

## 启动流程

### 1. 启动车辆，布设边缘云服务器

安装边缘云服务器，用网线连接路由器，电脑有线网口配置IP需要一段时间。等ifconfig命令显示有线网口有IPv4地址后，地址为192.168.10.100。如果地址最后为其他数字，则这个手动设置为静态IP，并改为192.168.10.100。

车端开机后连接到WIFI WAVLINK-A28C。

### 2. 时间同步
在边缘云服务器上，运行以下命令：
```bash
sudo systemctl restart chrony.service
```
在车端，运行以下命令：
```bash
sudo systemctl restart chrony.service
```
并在车端通过下面命令查看时间同步状态：
```bash
chronyc sourcestats             
```
在192.168.10.100一行的显示的同步时间单位为us时，说明时间同步成功。否则重启 chrony 服务。

### 3. 设置通讯IP

修改src/common/config/comm.json中对应计算终端的ip地址。

- 默认边缘云维护的ip地址为：192.168.10.100

- 长安车的ip地址为：192.168.10.120
 
- 合创车的ip地址为：192.168.10.110

### 4. 编译系统

在边缘云服务器上，运行以下命令：
```bash
cd scripts/center
bash center_compile.sh
```

在长安车端orin，运行以下命令：
```bash
cd scripts/rock
bash rock_compile.sh
```

在合创车端，运行以下命令：
```bash
cd scripts/hycan
bash hycan_compile.sh
```

### 5. 启动系统

#### 边缘云
在边缘云服务器上，运行以下命令：
```bash
bash center_launch.sh
```

#### 长安车
在长安车端先在xavier上运行以下命令，启动相机驱动：
```bash
cd ~/Desktop/CyberRock_IVFC/scripts
bash driver_xavier.sh
```

然后在右nuc上运行以下命令，启动GPS和IMU：
```bash
cd ~/Desktop/CyberRock_IVFC/scripts
bash driver_right.sh
```

最后在长安车orin的V2X_AEB目录，运行以下命令：
```bash
cd scripts/rock
bash rock_launch.sh
```


#### 合创车
在合创车端xavier，运行以下命令，启动相机驱动：
```bash
roscore
cd ~/Desktop/driver
bash camera_ros_driver.sh
```

然后继续在合创车端xavier，运行以下命令，启动GPS和IMU：
```bash
cd ~/projects/Cyberc3_platoon/scripts
bash driver_gps.sh
```

最后回到orinV2X_AEB目录，运行以下命令:
```bash
cd scripts/hycan
bash hycan_launch.sh
```
当车端以下信息时，说明系统启动成功：
```bash
[ INFO] [1634020007.000000000]: Connection to 192.168.10.100 has been established.
```
若车端提示以下消息，说明边缘云没有启动：
```bash
[ INFO] [1634020007.000000000]: Connecting to 192.168.10.100 ...
```

当服务器端提示以下信息时，说明系统启动成功：
```bash 
[ INFO] [1634020007.000000000]: Socket bind for center server success!
```

若服务器提示以下信息，说明有车辆连接到服务器：
```bash
[ INFO] [1634020007.000000000]: Connected from 192.168.10.110/192.168.10.120
[ INFO] [1634020007.000000000]: Receive image data from hycan/rock 
[ INFO] [1634020007.000000000]: Receive data length: xxx
[ INFO] [1634020007.000000000]: Time delay is xxx s
```

