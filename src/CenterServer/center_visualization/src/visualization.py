from hycan_msgs.msg import DetectionResults, Box3D
import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Quaternion
import tf.transformations as tf
import os
from numpy import cos, sin

def create_marker(id, position, orientation, scale, color=(0.0, 1.0, 0.0, 1.0)):
    marker = Marker()
    marker.header.frame_id = "world"
    marker.type = Marker.CUBE
    marker.id = id

    marker.pose.position = Point(*position)
    marker.pose.orientation = Quaternion(*orientation)
    marker.scale.x = scale[0]
    marker.scale.y = scale[1]
    marker.scale.z = scale[2]

    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.color.a = color[3]

    return marker

def create_arrow_marker(id,x,y,yaw, length, frame_id="world"):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.type = marker.ARROW
    marker.action = marker.ADD
    marker.id = id

    # 起点和终点设置
    start_point = Point()
    start_point.x = x
    start_point.y = y
    start_point.z = 0.0

    end_point = Point()
    end_point.x = x + length * cos(yaw)
    end_point.y = y + length * sin(yaw)
    end_point.z = 0.0

    marker.points.append(start_point)
    marker.points.append(end_point)

    quaternion = tf.quaternion_from_euler(0, 0, 0)
    marker.pose.orientation = Quaternion(*quaternion)

    # 设置箭头的颜色和透明度
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.color.a = 1.0

    # 设置箭头的尺度
    marker.scale.x = 0.1  # 箭头杆的宽度
    marker.scale.y = 0.2  # 箭头头部的宽度
    marker.scale.z = 0.0  # 2D箭头，不需要Z轴尺度

    return marker

def create_text_marker(id, text, x, y, z, frame_id="world"):
    text_marker = Marker()
    text_marker.id = id
    text_marker.header.frame_id = frame_id
    text_marker.header.stamp = rospy.Time.now()
    text_marker.type = Marker.TEXT_VIEW_FACING
    text_marker.action = Marker.ADD
    text_marker.pose.position.x = x
    text_marker.pose.position.y = y
    text_marker.pose.position.z = z
    text_marker.scale.z = 0.3  # Height of the text

    text_marker.color.r = 1.0
    text_marker.color.g = 1.0
    text_marker.color.b = 1.0
    text_marker.color.a = 1.0  # Opacity

    text_marker.text = text

    return text_marker

def delete_marker(id):
    marker = Marker()
    marker.id = id
    marker.action = Marker.DELETE
    return marker

class center_visualization:
    def __init__(self):
        rospy.init_node('center_visualization', anonymous=True)
        self.init_position = None
        self.prev_max_id = None
        self.prev_max_pre_id = None
        rospy.Subscriber('original_results', DetectionResults, self.original_detection_callback)
        rospy.Subscriber('fusion_results', DetectionResults, self.fusion_detection_callback)
        self.original_pub = rospy.Publisher('original_marker', MarkerArray, queue_size=10)
        self.fusion_pub = rospy.Publisher('fusion_marker', MarkerArray, queue_size=10)

        config_path = os.path.abspath(__file__).split('CenterServer')[0] + '/common/config/center_vis_config.rviz'
        os.system(f'rosrun rviz rviz -d {config_path}')



    def original_detection_callback(self, msg):

        vehicle_localization = msg.localization
        x, y, z = vehicle_localization.utm_x, \
            vehicle_localization.utm_y, \
            0.8
        if self.init_position is None:
            self.init_position = (x, y)
        person_markers = MarkerArray()
        num = len(msg.box3d_array)
        for i, box in enumerate(msg.box3d_array):
            id = box.id
            x, y, z = box.center_x, box.center_y, box.center_z
            x -= self.init_position[0]; y -= self.init_position[1]
            heading = box.heading
            height, width, length = box.height, box.width, box.length
            
            if id == -1:
                vehicle_id = create_text_marker(i + 2 * num + 1, 'rock', x, y, z + height / 2 + 0.1)
                person_markers.markers.append(vehicle_id)
                person_color = (1.0, 1.0, 0.0, 0.8)
            elif id == -2:
                vehicle_id = create_text_marker(i + 2 * num + 1, 'hycan', x, y, z + height / 2 + 0.1)
                person_markers.markers.append(vehicle_id)
                person_color = (1.0, 0.0, 1.0, 0.8)
            else:
                person_id = create_text_marker(i + 2 * num + 1, str(id), x, y, z + height / 2 + 0.1)
                person_markers.markers.append(person_id)

            if width > 2:
                color = (1.0, 0.0, 0.0, 0.8)
            else:
                color = person_color
            person_marker = create_marker(i + 1, (x, y, z), tf.quaternion_from_euler(0, 0, heading), (width, length, height), color=color)
            person_arrow = create_arrow_marker(i + num + 1, x, y, heading, 1.0)
            person_markers.markers.append(person_marker)
            person_markers.markers.append(person_arrow)

        prev_max_id = len(person_markers.markers) + 1
        if self.prev_max_id:
            for i in range(len(person_markers.markers) + 1, self.prev_max_id + 1):
                person_markers.markers.append(delete_marker(i))
        self.prev_max_id = prev_max_id
        
        # merge the two markers
        self.original_pub.publish(person_markers)
        rospy.loginfo("Visualization for original results published with {} boxes".format(len(person_markers.markers)))

    def fusion_detection_callback(self, msg):
        person_markers = MarkerArray()
        num = len(msg.box3d_array)
        for i, box in enumerate(msg.box3d_array):
            id = box.id
            x, y, z = box.center_x, box.center_y, box.center_z
            x -= self.init_position[0]; y -= self.init_position[1]
            heading = box.heading
            height, width, length = box.height, box.width, box.length
            person_marker = create_marker(i + 1, (x, y, z), tf.quaternion_from_euler(0, 0, heading), (width, length, height), color=(0.0, 0.0, 1.0, 0.8))
            person_arrow = create_arrow_marker(i + 1 + num, x, y, heading, 1.0)
            person_id = create_text_marker(i +1 + 2 * num , str(id), x, y, z + height / 2 + 0.1)
            person_markers.markers.append(person_marker)
            person_markers.markers.append(person_arrow)
            person_markers.markers.append(person_id)

        prev_max_pre_id = len(person_markers.markers) + 1

        if self.prev_max_pre_id:
            for i in range(len(person_markers.markers) + 1, self.prev_max_pre_id + 1):
                person_markers.markers.append(delete_marker(i))
        self.prev_max_pre_id = prev_max_pre_id

        self.fusion_pub.publish(person_markers)
        rospy.loginfo("Fusion results published with {} boxes".format(len(person_markers.markers)))



if __name__ == '__main__':
    vis = center_visualization()
    rospy.spin()
