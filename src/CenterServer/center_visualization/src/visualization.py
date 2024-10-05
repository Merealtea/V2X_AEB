from hycan_msgs.msg import DetectionResults, Box3D
import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Quaternion
import tf.transformations as tf
import os

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

class center_visualization:
    def __init__(self):
        rospy.init_node('center_visualization', anonymous=True)
        self.init_position = None
        rospy.Subscriber('original_results', DetectionResults, self.original_detection_callback)
        rospy.Subscriber('fusion_results', DetectionResults, self.fusion_detection_callback)
        self.original_pub = rospy.Publisher('original_marker', MarkerArray, queue_size=10)
        self.fusion_pub = rospy.Publisher('fusion_marker', MarkerArray, queue_size=10)
        config_path = os.path.abspath(__file__).split('CenterServer')[0] + '/common/config/center_vis_config.rviz'
        os.system(f'rosrun rviz rviz -d {config_path}')



    def original_detection_callback(self, msg):

        vehicle_bbox = MarkerArray()
        vehicle_localization = msg.localization
        x, y, z = vehicle_localization.utm_x, \
            vehicle_localization.utm_y, \
            0.8
        if self.init_position is None:
            self.init_position = (x, y)
        person_markers = MarkerArray()
        for i, box in enumerate(msg.box3d_array):
            x, y, z = box.center_x, box.center_y, box.center_z
            x -= self.init_position[0]; y -= self.init_position[1]
            heading = box.heading
            height, width, length = box.height, box.width, box.length
            if width > 2:
                color = (1.0, 0.0, 0.0, 0.8)
            else:
                color = (0.0, 1.0, 0.0, 0.8)
            person_marker = create_marker(i + 1, (x, y, z), tf.quaternion_from_euler(0, 0, heading), (width, length, height), color=color)
            person_markers.markers.append(person_marker)
        
        # merge the two markers
        vehicle_bbox.markers.extend(person_markers.markers)
        self.original_pub.publish(vehicle_bbox)
        rospy.loginfo("Visualization for original results published with {} boxes".format(len(vehicle_bbox.markers)))

    def fusion_detection_callback(self, msg):

        person_markers = MarkerArray()
        for i, box in enumerate(msg.boxes):
            x, y, z = box.center_x, box.center_y, box.center_z
            heading = box.heading
            height, width, length = box.height, box.width, box.length
            person_marker = create_marker(i + 1, (x, y, z), tf.quaternion_from_euler(0, 0, heading), (width, length, height), color=(0.0, 0.0, 1.0, 0.8))
            person_markers.markers.append(person_marker)

        self.fusion_pub.publish(person_markers)
        rospy.loginfo("Fusion results published with {} boxes".format(len(person_markers.markers)))



if __name__ == '__main__':
    vis = center_visualization()
    rospy.spin()
