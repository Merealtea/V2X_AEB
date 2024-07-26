# SJTU CyberC3 无人车队2.0
上海交通大学智能车实验室无人车队2.0代码仓库
***

## 调试方法
在线调试时，**切换至主目录下**，启动所有节点后，打开动态参数调节窗口  
`rosrun rqt_reconfigure rqt_reconfigure`  
该方式可以实时调整参数，但是下一次启动程序又会恢复默认。需要保存参数，则  
`rosparam dump src/control/params/rock.yaml #NODE`  
此处`#NODE`为节点名称，如 `/rock_vehicle_node/`, 可通过 `rosparam list`查询。
**保存参数功能已通过save_params.sh实现。**
***

## 硬件
- 头车：CyberE100
  - 工控机
    - 用户：g200
    - password: root
- 后车1：CyberRock
  - Xavier
    - 用户：nvidia
    - password: nvidia
  - NUC
    - password: cyber
***

## 车间通信配置
车间通信暂时基于路由WIFI，路由器放置在头车，通过TCP协议，**仅**与后车Xavier通信，不依赖ROS，**不污染后车内部局域网**。
- 路由器信息(放在前车CyberE100)
  - 主路由器（TPLINK）
    - SSID: CyberE100
    - password: CyberSmart220
    - 管理员账户：CyberE100
    - 管理员密码：Cyber220Smart
    - 网关：`192.168.30.1` （与出厂网关不一致）
- 局域网拓扑
  子网掩码: `255.255.255.0`
  IP范围: `192.168.30.2~254`
>+-->路由器网关: `192.168.30.1`  
|  
|  
+-->头车（网卡 enp0s31f6）: `192.168.30.2` （连接 Route 网络）
|  
|  
+-->后车 1 Xavier （网卡 wlan0）: `192.168.30.4`  
|  
- 多车时间同步
  chrony
***

## 软件算法
TensorRT 模型无法跨机器使用，生成和使用的环境必须保持一致
