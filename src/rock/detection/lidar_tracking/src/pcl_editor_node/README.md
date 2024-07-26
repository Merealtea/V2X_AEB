# PCL Editor
## Usage
- Start:
```shell
roslaunch tracking pcl_editor.launch
```
RVIZ and rqt will start
- Loop replay ros bag:
```shell
rosbag play -l xxx.bag
```
- Then move and spin pcl in rqt and scope rviz to adjust target position and pose.
- Then adjust selection box range to set crop area.
- Change file name in rqt. The target model will be save with new file name.
