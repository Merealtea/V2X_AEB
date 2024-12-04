import math
import numpy as np
import rospy
from visualization_msgs.msg import Marker, MarkerArray
import scipy

hycan_gps_topic = '/strong/fix'
hycan_heading_topic = '/strong/heading'
rock_gps_topic = '/Inertial/gps/fix'
rock_imu_topic = '/Inertial/imu/data'
rock_lidar_topic = '/driver/hesai/pandar'
hycan_lidar_topic = '/livox/lidar'

min_x , min_y = 1e9, 1e9
max_x, max_y = -1e9, -1e9
GLOBAL_ZERO_X = 351425.09269358893
GLOBAL_ZERO_Y = 3433830.3251591502

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

hycan_length = 4.6
hycan_width = 1.9
rock_length = 4.5
rock_width = 1.8
hycan_x_offset = 1.425
hycan_y_offset = 0.475  * 2
rock_x_offset = 0.96
rock_y_offset = 0

hycan_bounding_box = np.array([[hycan_length/2, hycan_width/2],
                            [hycan_length/2, -hycan_width/2],
                            [-hycan_length/2, -hycan_width/2],
                            [-hycan_length/2, hycan_width/2],
                            [hycan_length/2, hycan_width/2],]) + np.array([hycan_x_offset, hycan_y_offset])
rock_bounding_box = np.array([[rock_length/2, rock_width/2],
                            [rock_length/2, -rock_width/2],
                            [-rock_length/2, -rock_width/2],
                            [-rock_length/2, rock_width/2],
                            [rock_length/2, rock_width/2],]) + np.array([rock_x_offset, rock_y_offset])
    

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


def merge_msgs(msgs_list):
    merged_msgs = []
    for msgs in msgs_list:
        for topic, msg, t in msgs:
            merged_msgs.append([topic, msg, t])

    merged_msgs = sorted(merged_msgs, key=lambda x: x[2])
    return merged_msgs

def hycan_localization(gps, heading):
    hycan_lat, hycan_lon = gps.latitude, gps.longitude
    position = LLtoUTM(hycan_lat, hycan_lon, "32")
    yaw = heading.data 
    return position, yaw

def rock_localization(gps, imu):
    rock_lat, rock_lon = gps.latitude, gps.longitude
    position = LLtoUTM(rock_lat, rock_lon, "32")
    lat = gps.latitude
    lon = gps.longitude
    # Get Orientation
    utm = LLtoUTM(lat, lon, "32")

    heading = np.arctan2(2.0 * (imu.orientation.w * imu.orientation.z + imu.orientation.x * imu.orientation.y),   
                        1.0 - 2.0 * (imu.orientation.y * imu.orientation.y + imu.orientation.z * imu.orientation.z))
    return position, heading


def compute_convergence_angle(lon, lat, central_meridian):
    # 计算经度偏差
    delta_lon = math.radians(lon - central_meridian)
    # 计算收敛角
    convergence_angle = delta_lon * math.sin(math.radians(lat))
    return math.degrees(convergence_angle)


def transform_bbox(bbox, position, yaw):
    rotation_matrix = np.array([[np.cos(yaw), -np.sin(yaw)],
                                [np.sin(yaw), np.cos(yaw)]])
    rotated_box = np.dot(bbox, rotation_matrix.T)
    rotated_box += np.array(position).reshape(1, 2)
    return rotated_box

def create_marker(id, color, box):
    marker = Marker()
    marker.id = id
    marker.header.frame_id = "map"
    marker.type = marker.CUBE
    marker.action = marker.ADD
    marker.scale.x = box.length  # 长度
    marker.scale.y = box.width  # 宽度
    marker.scale.z = box.height  # 高度，设为一个较小的值
    # marker.lifetime = rospy.Duration(0.2)

    marker.header.stamp = rospy.Time.now()
    marker.pose.position.x = box.center_x 
    marker.pose.position.y = box.center_y 
    marker.pose.position.z = box.center_z

    # 设置方向
    # 使用scipy将欧拉角转换为四元数
    orientation = scipy.spatial.transform.Rotation.from_euler('zyx', [box.heading, 0, 0]).as_quat()
    marker.pose.orientation.x = orientation[0]
    marker.pose.orientation.y = orientation[1]
    marker.pose.orientation.z = orientation[2]
    marker.pose.orientation.w = orientation[3]

    # 设置颜色
    marker.color.a = 0.4  # Alpha, 1表示完全不透明
    marker.color.r = color[0]  # 红色
    marker.color.g = color[1]  # 绿色
    marker.color.b = color[2]  # 蓝色
    return marker