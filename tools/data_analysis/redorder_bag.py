from copy import deepcopy
import rosbag  
import rospy
from sensor_msgs.msg import NavSatFix, CompressedImage
from cyber_msgs.msg import Heading
import numpy as np
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber

bag_path = "/mnt/pool1/2024-09-11.bag"

bag = rosbag.Bag(bag_path, 'r')
gps_fix_topic = '/strong/fix'
heading_topic = '/strong/heading'

new_bag_path = "/mnt/pool1/new_bag.bag"
new_bag = rosbag.Bag(new_bag_path, 'w')

def camera_callback(msg1, msg2, msg3, msg4):
    new_gps = NavSatFix()
    new_heading = Heading()
    new_bag.write(gps_fix_topic, new_gps, msg1.header.stamp)
    new_bag.write(heading_topic, new_heading, msg1.header.stamp)
    new_bag.write(front_camera_topic, msg1, msg1.header.stamp)
    new_bag.write(back_camera_topic, msg2, msg2.header.stamp)
    new_bag.write(right_camera_topic, msg3, msg3.header.stamp)
    new_bag.write(left_camera_topic, msg4, msg4.header.stamp)

    # cap.write(cv_image)


if __name__ == "__main__":
    rospy.init_node('merge_bag')
    front_camera_topic = "/miivii_gmsl_ros/camera1/compressed"
    back_camera_topic = "/miivii_gmsl_ros/camera2/compressed"
    right_camera_topic = "/miivii_gmsl_ros/camera3/compressed"
    left_camera_topic = "/miivii_gmsl_ros/camera4/compressed"

    cam_1 = Subscriber(front_camera_topic, CompressedImage)
    cam_2 = Subscriber(back_camera_topic, CompressedImage)
    cam_3 = Subscriber(right_camera_topic, CompressedImage)
    cam_4 = Subscriber(left_camera_topic, CompressedImage)

    ts = ApproximateTimeSynchronizer([cam_1, cam_2, cam_3, cam_4], 10, 0.1)

    ts.registerCallback(camera_callback)

    for topic, msg, t in bag.read_messages(topics=[front_camera_topic, back_camera_topic, right_camera_topic, left_camera_topic]):
        if topic == front_camera_topic:
            cam_1.signalMessage(msg)
        elif topic == back_camera_topic:
            cam_2.signalMessage(msg)
        elif topic == right_camera_topic:
            cam_3.signalMessage(msg)
        elif topic == left_camera_topic:
            cam_4.signalMessage(msg)

    bag.close()
    new_bag.close()