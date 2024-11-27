import rosbag
import rospy
from hycan_msgs.msg import DetectionResults, Box3D
import cv2
import argparse
from message_filters import ApproximateTimeSynchronizer, Subscriber
import numpy as np

def parse_args():
    parser = argparse.ArgumentParser(description='Visualize detection results')
    parser.add_argument('--bag', type=str, required=True, help='Path to the rosbag file')
    args = parser.parse_args()
    return args


def merge_callback(hycan_msgs, rock_msgs, fusion_msgs):
    hycan_localization = hycan_msgs.localization
    rock_localization = rock_msgs.localization
    
    hycan_boxes = hycan_msgs.box3d_array
    rock_boxes = rock_msgs.box3d_array
    fusion_boxes = fusion_msgs.box3d_array

    width = 200
    height = 200
    res = 0.2

    track_bev = np.zeros((height, width, 3), dtype=np.uint8)
    

    

if __name__ == "__main__":
    bag = rosbag.Bag("2021-09-30_15-11-28.bag")
    hycan_det_topic = '/hycan/detection_results'
    rock_det_topic = '/rock/detection_results'
    fusion_det_topic = '/final_fusion_results'

    hycan_det_sub = rospy.Subscriber(hycan_det_topic, DetectionResults)
    rock_det_sub = rospy.Subscriber(rock_det_topic, DetectionResults)
    fusion_det_sub = rospy.Subscriber(fusion_det_topic, DetectionResults)

    for topic, msg, t in bag.read_messages(topics=[hycan_det_topic, rock_det_topic, fusion_det_topic]):
        if topic == hycan_det_topic:
            