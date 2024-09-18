# 头车V2V驱动模块
## 多车时间同步
通过NTP局域网同步
1. 安装ntp与ntpdate（前车后车均安装）
`sudo apt-get install ntp ntpdate`
2. 配置NTP服务器（暂定后车1（CyberRock）的Xavier为NTP服务器）
打开Xavier的ntp配置文件
`sudo gedit /etc/ntp.conf`
在文件末尾加上
`restrict 192.168.30.0 mask255.255.255.0 nomodify notrap    # 使 192.168.30.x网段能够使用该机器的NTP服务`
`server 127.127.1.0 # local clock`
`fudge 127.127.1.0 stratum 10`
保存，关闭防火墙，
`sudo ufw disable`
然后重启ntp
`sudo service ntp restart`
3. 配置客户端（头车作为客户端）
打开ntp配置文件
`sudo gedit /etc/ntp.conf`
添加
`server 192.168.30.4 prefer # 加入头车IP`
保存，关闭防火墙
`sudo ufw disable`
然后重启ntp服务
`sudo service restart`
4. 若没有时间同步，保持以上配置不变，在后车手动进行同步，首先关闭后车ntp
`sudo service ntp stop`
然后手动同步
`sudo ntpdate 192.168.30.4`
5. 手动同步可以使用cron，定时执行
首先安装cron
`sudo apt-get install cron`
启动cron
`sudo service cron start`
编辑计划
`crontab -e`
然后输入
`* * * * * sudo ntpdate 192.168.30.4`
即每分钟执行一次时间同步

