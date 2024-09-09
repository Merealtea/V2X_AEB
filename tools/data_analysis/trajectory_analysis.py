from copy import deepcopy
import rosbag  
import rospy
from sensor_msgs.msg import Image
from cyber_msgs.msg import Heading
import numpy as np
import matplotlib.pyplot as plt
from hycan_msgs.msg import DetectionResults
from message_filters import ApproximateTimeSynchronizer, Subscriber
from pyproj import Transformer
import numpy as np
import math
import open3d as o3d

GLOBAL_ZERO_X = 351425.09269358893
GLOBAL_ZERO_Y = 3433830.3251591502

# 读取PCD文件
pcd = o3d.io.read_point_cloud("/mnt/pool1/outside_parking/color_map.pcd")
length = 4.6
width = 1.9
height = 1.65

points = np.array(
    [[length/2, width/2, 0], 
    [length/2, -width/2, 0],
    [-length/2, -width/2, 0],
    [-length/2, width/2, 0],
    [length/2, width/2, height],
    [length/2, -width/2, height],
    [-length/2, -width/2, height],
    [-length/2, width/2, height]])

# 创建矩形框
lines = [
    [0, 1], [1, 2], [2, 3], [3, 0],  # 底面四条边
    [4, 5], [5, 6], [6, 7], [7, 4],  # 顶面四条边
    [0, 4], [1, 5], [2, 6], [3, 7]   # 连接上下面的四条边
]


# From wgs84 to utm
transformer = Transformer.from_crs("epsg:4326", "epsg:32632")

bounding_box = np.array([[length/2, width/2],
                            [length/2, -width/2],
                            [-length/2, -width/2],
                            [-length/2, width/2],
                            [length/2, width/2],])
    
bounding_boxes = []
yaws = []

i = 0
max_frame =400
min_x , min_y = 1e9, 1e9
max_x, max_y = -1e9, -1e9

RADIAN_PER_DEGREE = np.pi / 180
DEGREES_PER_RADIAN = 180 / np.pi
central_meridian = 120

new_bag = None
traj = []

# WGS84 Parameters

WGS84_A = 6378137.0          # major axis
WGS84_B = 6356752.31424518   # minor axis
WGS84_F = 0.0033528107       # ellipsoid flattening
WGS84_EP = 0.0820944379      # second eccentricity

WGS84_E = 0.0818191908    # first eccentricity
# UTM Parameters
UTM_K0 = 0.9996          # scale factor
UTM_FE = 500000.0        # false easting
UTM_FN_N = 0.0           # false northing on north hemisphere
UTM_FN_S = 10000000.0    # false northing on south hemisphere
UTM_E2 = (WGS84_E * WGS84_E)         # e^2
UTM_E4 = (UTM_E2 * UTM_E2)           # e^4
UTM_E6 = (UTM_E4 * UTM_E2)           # e^6
UTM_EP2 = (UTM_E2 / (1 - UTM_E2))    # e'^2

LongOriginCustom = 121.43371749 

RADIANS_PER_DEGREE = math.pi / 180.0
DEGREES_PER_RADIAN = 180.0 / math.pi

idx= 0


def compute_convergence_angle(lon, lat, central_meridian):
    # 计算经度偏差
    delta_lon = math.radians(lon - central_meridian)
    # 计算收敛角
    convergence_angle = delta_lon * math.sin(math.radians(lat))
    return math.degrees(convergence_angle)

def callback(hycan_gps, hycan_heading): #, rock_gps, rock_imu):
    global new_bag, points, LongOriginCustom, RADIANS_PER_DEGREE, DEGREES_PER_RADIAN, traj, idx 
    idx += 1
    if idx % 30:
        return

    hycan_lat = hycan_gps.latitude
    hycan_lon = hycan_gps.longitude
    # Get Orientation
    delta_lon = (hycan_lat - LongOriginCustom) * RADIANS_PER_DEGREE
    convergence_angle = math.pi / 4# delta_lon * math.sin(hycan_lat * RADIANS_PER_DEGREE) 

    corrected_heading = hycan_heading.data + convergence_angle - math.pi / 2
    utm = LLtoUTM(hycan_lat, hycan_lon, "32")
    vehicle_points = np.dot(deepcopy(points), np.array([[np.cos(corrected_heading), -np.sin(corrected_heading), 0],
                                                        [np.sin(corrected_heading), np.cos(corrected_heading), 0],
                                                        [0, 0, 1]]))

    vehicle_points = np.array([[utm[0], utm[1], 0]]) + vehicle_points

    rospy.loginfo("Corrected Heading: {} radians".format(corrected_heading))

    
    # Create a new point cloud
    traj.append(vehicle_points)

def LLtoUTM( Lat, Long, UTMZone):
    a = WGS84_A
    eccSquared = UTM_E2
    k0 = UTM_K0
    # Make sure the longitude is between -180.00 .. 179.9
    LongTemp = (Long + 180) - int((Long + 180) / 360) * 360 - 180 

    LatRad = Lat * RADIANS_PER_DEGREE 
    LongRad = LongTemp * RADIANS_PER_DEGREE 
 
    ZoneNumber = int((LongTemp + 180) / 6) + 1 

    if (Lat >= 56.0  and Lat < 64.0  and LongTemp >= 3.0  and LongTemp < 12.0):
        ZoneNumber = 32 
   # cout << "ZoneNumber: "<< ZoneNumber << endl 

   # Special zones for Svalbard
    if (Lat >= 72.0  and Lat < 84.0) :
        if (LongTemp >= 0.0  and LongTemp < 9.0):
            ZoneNumber = 31 
        elif (LongTemp >= 9.0  and LongTemp < 21.0):
            ZoneNumber = 33 
        elif (LongTemp >= 21.0  and LongTemp < 33.0) :
            ZoneNumber = 35 
        elif (LongTemp >= 33.0  and LongTemp < 42.0) :
            ZoneNumber = 37 
   # +3 puts origin in middle of zone

   #---------------------modified LongOrigin by wlh
   # 20160518----------------------------------------
    LongOrigin = (ZoneNumber - 1) * 6 - 180 + 3 
   #		LongOrigin = LongOriginCustom 
    LongOriginRad = LongOrigin * RADIANS_PER_DEGREE 
   # cout << "Letter: "<< UTMLetterDesignator(Lat) << endl 
   # compute the UTM Zone from the latitude and longitude
   #        snprintf(UTMZone, 4, "%d%c", ZoneNumber, UTMLetterDesignator(Lat)) 

    eccPrimeSquared = (eccSquared) / (1 - eccSquared) 

    N = a / math.sqrt(1 - eccSquared * math.sin(LatRad) * math.sin(LatRad)) 
    T = math.tan(LatRad) * math.tan(LatRad) 
    C = eccPrimeSquared * math.cos(LatRad) * math.cos(LatRad) 
    A = math.cos(LatRad) * (LongRad - LongOriginRad) 

    M = a * \
      ((1 - eccSquared / 4 - 3 * eccSquared * eccSquared / 64 -
        5 * eccSquared * eccSquared * eccSquared / 256) *
           LatRad -
       (3 * eccSquared / 8 + 3 * eccSquared * eccSquared / 32 +
        45 * eccSquared * eccSquared * eccSquared / 1024) *
           math.sin(2 * LatRad) +
       (15 * eccSquared * eccSquared / 256 +
        45 * eccSquared * eccSquared * eccSquared / 1024) *
           math.sin(4 * LatRad) -
       (35 * eccSquared * eccSquared * eccSquared / 3072) * math.sin(6 * LatRad)) 

    UTMEasting = float(k0 * N *
                   (A + (1 - T + C) * A * A * A / 6 +
                    (5 - 18 * T + T * T + 72 * C - 58 * eccPrimeSquared) * A *
                        A * A * A * A / 120) +
               500000.0) 

    UTMNorthing = float(k0 * (M + N * math.tan(LatRad) *
                             (A * A / 2 +
                              (5 - T + 9 * C + 4 * C * C) * A * A * A * A / 24 +
                              (61 - 58 * T + T * T + 600 * C -
                               330 * eccPrimeSquared) *
                                  A * A * A * A * A * A / 720))) 
    if (Lat < 0):
        UTMNorthing += 10000000.0    # 10000000 meter offset for southern hemisphere

    UTMNorthing -= GLOBAL_ZERO_Y 
    UTMEasting -= GLOBAL_ZERO_X 
    return [UTMEasting, UTMNorthing]


if __name__ == "__main__":
    rospy.init_node('localization_analysis')

    # 创建两个Subscriber对象
    # rock_imu_sub = Subscriber("/Inertial/imu/data", Imu)
    # rock_gps_sub = Subscriber('/Inertial/gps/fix', NavSatFix)

    hycan_imu_sub = Subscriber("/strong/fix", Imu)
    hycan_gps_sub = Subscriber('/strong/heading', Heading)

    hycan_bag_path = "/mnt/pool1/cameras_undi.bag"
    # rock_bag_path = "/mnt/pool1/outside_parking/both/2024-09-02-20-38-51.bag"
    
    # 打开两个rosbag文件
    hycan_bag = rosbag.Bag(hycan_bag_path, 'r')
    # rock_bag = rosbag.Bag(rock_bag_path, 'r') 

    # new_bag = rosbag.Bag("/mnt/pool1/outside_parking/both/merged.bag", 'w')

    # 创建ApproximateTimeSynchronizer
    sync = ApproximateTimeSynchronizer([hycan_gps_sub, hycan_imu_sub],\
                                        queue_size=10, slop=0.1)
    sync.registerCallback(callback)

    # 合并两个bag的消息迭代器
    # rock_bag_msgs = rock_bag.read_messages(topics=['/Inertial/gps/fix', '/Inertial/imu/data'])
    hycan_bag_msgs = hycan_bag.read_messages(topics=['/strong/fix', '/strong/heading'])
    # merged_msgs = sorted(list(rock_bag_msgs) + list(hycan_bag_msgs), key=lambda x: x[2])  # 按时间戳排序

    # 读取bag文件并触发callbacks
    for topic, msg, t in hycan_bag_msgs:
        if topic == '/strong/fix':
            hycan_gps_sub.signalMessage(msg)
        elif topic == '/strong/heading':
            hycan_imu_sub.signalMessage(msg)
        # elif topic == '/Inertial/gps/fix':
        #     rock_gps_sub.signalMessage(msg)
        # elif topic == '/Inertial/imu/data':
        #     rock_imu_sub.signalMessage(msg)
    
    traj = np.array(traj).reshape(-1, 3)
    combined_points = np.vstack((np.asarray(pcd.points), traj))

    # 创建新的点云对象
    new_pcd = o3d.geometry.PointCloud()
    new_pcd.points = o3d.utility.Vector3dVector(combined_points)

    # 保存新的PCD文件
    o3d.io.write_point_cloud("./new_pointcloud_with_rectangle.pcd", new_pcd)

    print("Point cloud with rectangle vertices saved successfully.")