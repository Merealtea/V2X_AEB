from copy import deepcopy
import rosbag  
import rospy
from sensor_msgs.msg import Image
import numpy as np
from cv_bridge import CvBridge

if __name__ == "__main__":
    rospy.init_node('merge_bag')
    hycan_bag_path = "/mnt/pool1/outside_parking/hycan/2024-08-01_09-34-15.bag"
    new_hycan_bag_path = "/mnt/pool1/outside_parking/hycan/2024-08-01_09-34-15_send.bag"
    # 打开两个rosbag文件
    bag = rosbag.Bag(hycan_bag_path, 'r')
    new_bag = rosbag.Bag(new_hycan_bag_path, 'w')
    front_topic =  "/miivii_gmsl_ros/camera1/compressed"
    back_topic = "/miivii_gmsl_ros/camera2/compressed"
    right_topic = "/miivii_gmsl_ros/camera3/compressed"
    left_topic = "/miivii_gmsl_ros/camera4/compressed"

    bridge = CvBridge()

    for topic, msg, t in bag.read_messages(topics=['/Hycan/processed_images']):
        new_msg = deepcopy(msg)
        recv_stamp = msg.header.stamp
        send_stamp = msg.image_front.header.stamp
        new_msg.header.stamp = send_stamp
        new_msg.image_front.header.stamp = recv_stamp
        new_bag.write('/Hycan/processed_images', new_msg, send_stamp)
        front_image = np.frombuffer(msg.image_front.data, dtype=np.uint8).reshape((480, 640, 3))
        back_image = np.frombuffer(msg.image_back.data, dtype=np.uint8).reshape((480, 640, 3))
        left_image = np.frombuffer(msg.image_left.data, dtype=np.uint8).reshape((480, 640, 3))
        right_image = np.frombuffer(msg.image_right.data, dtype=np.uint8).reshape((480, 640, 3))
        front_image = bridge.cv2_to_imgmsg(front_image, encoding="bgr8")
        back_image = bridge.cv2_to_imgmsg(back_image, encoding="bgr8")
        left_image = bridge.cv2_to_imgmsg(left_image, encoding="bgr8")
        right_image = bridge.cv2_to_imgmsg(right_image, encoding="bgr8")
        
        front_image.header.stamp = send_stamp
        back_image.header.stamp = send_stamp
        left_image.header.stamp = send_stamp
        right_image.header.stamp = send_stamp

        new_bag.write(front_topic, front_image, send_stamp)
        new_bag.write(back_topic, back_image, send_stamp)
        new_bag.write(left_topic, left_image, send_stamp)
        new_bag.write(right_topic, right_image, send_stamp)
        print(f"Receive time: {recv_stamp}, Send time: {send_stamp}")

    bag.close()
    new_bag.close()
