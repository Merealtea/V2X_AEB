# 维特智能WTGAHRS3 GPS模块驱动 (Modbus协议)
## 使用前准备事项
- 安装python依赖库 `pip install pyserial modbus_tk`
- 复制`scripts/99-usb-serial.rules`至`/etc/udev/rules.d/`目录下，并重新插拔usb转串口模块
   - 可能需根据`lsusb`中的模块ID对文件进行修改
- 修改`wit_gps.launch`中串口号与波特率
## 发布topics
- /wit/gps_fix sensor_msgs::NavSatFix GPS组合导航结果
- /wit/imu sensor_msgs::Imu Imu模块数据
