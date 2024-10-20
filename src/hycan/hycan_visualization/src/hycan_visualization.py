import rospy 
import numpy as np
import cv2
from sensor_msgs.msg import CompressedImage
from message_filters import ApproximateTimeSynchronizer, Subscriber
import sys
import os
from hycan_msgs.msg import DetectionResults, Box3D

src_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
sys.path.append(src_path)
sys.path.append(os.path.join(src_path))
sys.path.append(src_path.split('hycan')[0])

from common.Mono3d.configs.FisheyeParam.cam_model import CamModel
from common.Mono3d.tools.utilities import calculate_corners, plot_rect3d_on_img

class ImageSubscriber:
    def __init__(self):
        rospy.init_node('image_subscriber', anonymous=True)
        front_sub = Subscriber('/miivii_gmsl_ros/camera1/compressed', CompressedImage)
        back_sub = Subscriber('/miivii_gmsl_ros/camera2/compressed', CompressedImage)
        right_sub = Subscriber('/miivii_gmsl_ros/camera3/compressed', CompressedImage)
        left_sub = Subscriber('/miivii_gmsl_ros/camera4/compressed', CompressedImage)

        if not os.path.exists(os.path.join(src_path, '../debug')):
            os.makedirs(os.path.join(src_path, '../debug'))
            rospy.loginfo("Created {} directory".format(os.path.join(src_path, 'debug')))

        detection_sub = rospy.Subscriber('hycan_detection_results', DetectionResults, self.detection_callback)

        self.ts = ApproximateTimeSynchronizer([front_sub, back_sub, right_sub, left_sub], queue_size=1, slop=0.1)
        self.ts.registerCallback(self.callback)

        self.image_sequence = {}
        self.cam_models = {}
        for direction in ["front", "back", "right", "left"]:
            self.image_sequence[direction] = []
            self.cam_models[direction] = CamModel(direction, "Hycan")

    def callback(self, front, back, right, left):
        front_img = cv2.imdecode(np.frombuffer(front.data, np.uint8), cv2.IMREAD_COLOR)
        back_img = cv2.imdecode(np.frombuffer(back.data, np.uint8), cv2.IMREAD_COLOR)
        right_img = cv2.imdecode(np.frombuffer(right.data, np.uint8), cv2.IMREAD_COLOR)
        left_img = cv2.imdecode(np.frombuffer(left.data, np.uint8), cv2.IMREAD_COLOR)

        self.image_sequence["front"].append(front_img)
        self.image_sequence["back"].append(back_img)
        self.image_sequence["right"].append(right_img)
        self.image_sequence["left"].append(left_img)

        if len(self.image_sequence["front"]) > 4:
            for direction in ["front", "back", "right", "left"]:
                self.image_sequence[direction].pop(0)

    def detection_callback(self, msg):
        box_array = []
        rospy.loginfo("Received {} boxes for visualization".format(msg.num_boxes))
        for i in range(msg.num_boxes):
            box = [msg.box3d_array[i].center_x, msg.box3d_array[i].center_y, msg.box3d_array[i].center_z,
                     msg.box3d_array[i].width, msg.box3d_array[i].length, msg.box3d_array[i].height,
                     msg.box3d_array[i].heading]
            box_array.append(box)
        box_array = np.array(box_array)

        concat_img = np.zeros((720 * 2, 1280 * 2, 3), dtype=np.uint8)

        for direction in ["front", "back", "right", "left"]:
            bboxes = []
            img = self.image_sequence[direction][0]
            cam_model = self.cam_models[direction]

            depth = cam_model.world2cam(box_array[:, :3].T).T[:, 0]
            box_cam = box_array[depth > 0.05]
            corners = calculate_corners(box_cam).reshape(-1, 3)
            corners = cam_model.world2cam(corners.T)
            corners[0][corners[0] < 0.05] = 0.05

            corners = corners.T.reshape(-1, 8, 3)
            for corner in corners:
                pixel_uv = cam_model.cam2image(corner.T).T
                bboxes.append(pixel_uv)
            img = plot_rect3d_on_img(img, len(bboxes), bboxes, color=(0, 0, 255))
            if direction == "front":
                concat_img[:720, :1280] = img
            elif direction == "back":
                concat_img[:720, 1280:] = img
            elif direction == "right":
                concat_img[720:, 1280:] = img
            elif direction == "left":
                concat_img[720:, :1280] = img
        cv2.imwrite(os.path.join(src_path, '../debug', '{:6f}.jpg'.format(msg.image_stamp.to_sec())), concat_img)

if __name__ == '__main__':
    image_subscriber = ImageSubscriber()
    rospy.spin()