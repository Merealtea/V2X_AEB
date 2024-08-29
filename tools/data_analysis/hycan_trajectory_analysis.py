from copy import deepcopy
import rosbag  
import rospy
from sensor_msgs.msg import Image
from cyber_msgs.msg import Heading
import numpy as np
import matplotlib.pyplot as plt
from sensor_msgs.msg import NavSatFix
from message_filters import ApproximateTimeSynchronizer, Subscriber
from pyproj import Transformer
import numpy as np
import math

# From wgs84 to utm
transformer = Transformer.from_crs("epsg:4326", "epsg:32632")
length = 4.6
width = 1.9
bounding_box = np.array([[length/2, width/2],
                            [length/2, -width/2],
                            [-length/2, -width/2],
                            [-length/2, width/2],
                            [length/2, width/2],])
    
bounding_boxes = []
yaws = []
points = []
i = 0
max_frame =400
min_x , min_y = 1e9, 1e9
max_x, max_y = -1e9, -1e9

RADIAN_PER_DEGREE = np.pi / 180
DEGREES_PER_RADIAN = 180 / np.pi
central_meridian = 120

def compute_convergence_angle(lon, lat, central_meridian):
    # 计算经度偏差
    delta_lon = math.radians(lon - central_meridian)
    # 计算收敛角
    convergence_angle = delta_lon * math.sin(math.radians(lat))
    return math.degrees(convergence_angle)

def callback(gps, heading):
    global i, min_x, min_y, max_x, max_y,central_meridian
    position = transformer.transform(gps.latitude, gps.longitude)

    # lon1 = last_gps.longitude * RADIANS_PER_DEGREE;
    # lat1 = last_gps.latitude * RADIANS_PER_DEGREE;
    # lon2 = current_gps.longitude * RADIANS_PER_DEGREE;
    # lat2 = current_gps.latitude * RADIANS_PER_DEGREE;

    # dLon = lon2 - lon1;
    # y = sin(dLon) * cos(lat2);
    # x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);
    # gps_heading = atan2(y, x);

    # corrected_heading = gps_heading + imu_yaw;
    # // Convert radians to degrees
    # corrected_heading = corrected_heading * DEGREES_PER_RADIAN;

    yaw = heading.data + compute_convergence_angle(gps.longitude, gps.latitude, central_meridian) - (np.pi / 2)
    rotation_matrix = np.array([[np.cos(yaw), -np.sin(yaw)],
                                [np.sin(yaw), np.cos(yaw)]])
    
    rotated_box = np.dot(bounding_box, rotation_matrix.T)
    rotated_box += np.array(position).reshape(1, 2)
    
    i += 1

    if i > 0 and i < 350:
        if min_x > position[0]:
            min_x = position[0]
        if min_y > position[1]:
            min_y = position[1]
        if max_x < position[0]:
            max_x = position[0]
        if max_y < position[1]:
            max_y = position[1]
        yaws.append(yaw)
        points.append(position)
        bounding_boxes.append(rotated_box)
            # import pdb; pdb.set_trace()
    


if __name__ == "__main__":
    rospy.init_node('hycan_bag')
    hycan_bag_path = "/mnt/pool1/cameras_undi.bag"
    
    # 打开两个rosbag文件
    bag = rosbag.Bag(hycan_bag_path, 'r')

    gps_topic = '/strong/fix'
    heading_topic = '/strong/heading'

     # 创建message_filters的Subscriber对象
    subscribers = [Subscriber(gps_topic, NavSatFix),
                   Subscriber(heading_topic, Heading)]
    
    # 创建ApproximateTimeSynchronizer对象
    ts = ApproximateTimeSynchronizer(subscribers, 10, 0.1)
    ts.registerCallback(callback)

    # 读取bag文件并触发callbacks
    for topic, msg, t in bag.read_messages(topics=[gps_topic, heading_topic]):
        for sub in subscribers:
            if topic == sub.name:
                sub.signalMessage(msg)

    bag.close()

    fig, ax = plt.subplots()
    ax.set_aspect('equal', adjustable='box')

    ax.set_ylim(min_y-10, max_y+10)        
    ax.set_xticks(np.arange(min_x-10, max_x+10, 1))
    ax.set_yticks(np.arange(min_y-10, max_y+10, 1))
    
    points= np.array(points)
    # 绘制轨迹
    # plt.plot(points[:, 0], points[:, 1], label="Trajectory")

    # # 在每个点上绘制表示朝向的箭头
    # for i in range(len(points)):
    #     x, y = points[i]
    #     angle = yaws[i]
    #     # 使用箭头来表示朝向
    #     dx = np.cos(angle) * 0.5  # 箭头的x分量
    #     dy = np.sin(angle) * 0.5  # 箭头的y分量
    #     plt.arrow(x, y, dx, dy, head_width=0.2, head_length=0.2, fc='r', ec='r')

    for box in bounding_boxes:
        ax.plot(box[:, 0], box[:, 1], 'r')
    plt.savefig('bounding_box.png', dpi = 600)