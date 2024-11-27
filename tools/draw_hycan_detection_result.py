import rosbag
import rospy
import cv2
import numpy as np
from hycan_msgs.msg import DetectionResults
from message_filters import ApproximateTimeSynchronizer, Subscriber
import os
import sys
from sensor_msgs.msg import CompressedImage
src_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(src_path)
sys.path.append(os.path.join(src_path,'src'))
import matplotlib.pyplot as plt
from common.Mono3d.configs.FisheyeParam.cam_model import CamModel
from common.Mono3d.configs.FisheyeParam.lidar_model import Lidar_transformation
from common.Mono3d.tools.utilities import calculate_corners, plot_rect3d_on_img
import argparse

def parse_args():
    parser = argparse.ArgumentParser(description='Draw detection results')
    parser.add_argument('--rosbag', type=str, help='Path to the rosbag file')
    parser.add_argument('--vehicle', type=str, help='Vehicle name')
    return parser.parse_args()

vehicle = None
cam_models = {}
rear_offset = 0
vehicle_length = 0
vehicle_width = 0
video_writer = None 
bev_video_writer = None
combined_video = None
tracks = {}

lidar_model = None

colors = {
    0 : (255, 0, 0),
    1 : (0, 255, 0),
    2 : (0, 0, 255),
    3 : (255, 255, 0),
    4 : (0, 255, 255),
    5 : (255, 0, 255),
    6 : (128, 128, 255),
    7 : (0, 0, 0),
    8 : (128, 128, 128),
    9 : (128, 0, 0),
    10 : (128, 128, 0),

}

def get_rotated_rectangle(x_center, y_center, width, length, angle):
    corners = np.array([[-width/2, -length/2],
                        [width/2, -length/2],
                        [width/2, length/2],
                        [-width/2, length/2]])
    rot_mat = np.array([[np.cos(angle), -np.sin(angle)],
                        [np.sin(angle), np.cos(angle)]])
    rotated_corners = corners @ rot_mat.T
    rotated_corners[:, 0] += x_center + 100
    rotated_corners[:, 1] += y_center + 100
    rotated_corners[:, [1, 0]] = rotated_corners[:, [0, 1]]
    return rotated_corners

def draw_boxes_callback(front_cam,
                        back_cam,
                        right_cam,
                        left_cam,
                        det_res,
                        track_res):
    global video_writer, tracks, bev_video_writer, rear_offset, vehicle_length, vehicle_width, \
            cam_models, colors, combined_video, lidar_model, vehicle
    current_track_ids = [box.id for box in track_res.box3d_array]
    fig, ax = plt.subplots(1, 1, figsize=(10, 10))
    width = 200; height = 200
    res = 0.1
    box_array = np.array([[box.center_x, box.center_y, 
                  box.width, box.length,  box.heading] 
                    for box in track_res.box3d_array])
    
    localization = track_res.localization
    utm_x, utm_y, heading = localization.utm_x, localization.utm_y, localization.heading
    if vehicle != "Hycan":
        heading += np.pi 
    if len(box_array) > 0:
    
        box_array[:, :2] = box_array[:, :2] - np.array([utm_x, utm_y])
        inv_mat = np.array([[np.cos(heading), np.sin(heading)],
                            [-np.sin(heading), np.cos(heading)]])
        box_array[:, :2] = (inv_mat @ box_array[:, :2].T).T
        box_array[:, 4] = box_array[:, 4] - heading

        if vehicle != "Hycan":
            box_array[:, 1] += 0.5

        # if vehicle != "Hycan":
        #     box_array[:, :3] = lidar_model.rear_to_lidar(box_array[:, :3].T).T
        
    print("current track ids: {} with {} boxes".format(current_track_ids, len(box_array)))
    for track_id, box in zip(current_track_ids, box_array):
        if track_id not in tracks:
            tracks[track_id] = []
        tracks[track_id].append(box)
        print("track id: {} with {} boxes".format(track_id, len(tracks[track_id])))

    vehicle = (np.array([rear_offset, 0.0, vehicle_length, vehicle_width]) / res).astype(int)
    vehicle = get_rotated_rectangle(vehicle[0], vehicle[1], vehicle[2], vehicle[3], 0)
    polygon = plt.Polygon(vehicle, fill=None, closed = True, edgecolor='r')
    ax.add_patch(polygon)

    for track_id in current_track_ids:
        track = np.array(tracks[track_id][-4:][::-1])
        track[:, :4] = track[:, :4] / res
        color = colors[track_id % len(colors)] 
        color = [c / 255 for c in color]
        alpha = 1
        for box in track:
            box = get_rotated_rectangle(box[0], box[1], box[2], box[3], box[4])
            polygon = plt.Polygon(box, fill=True, closed = True, color=color, alpha=alpha)
            ax.add_patch(polygon)    
            alpha -= 0.2
    ax.set_xlim(0, width)
    ax.set_ylim(0, height)
    plt.gca().invert_xaxis()
    ax.set_aspect('equal', adjustable='box')
    # save as np array
    fig.canvas.draw()
    plt.close(fig)

    data = np.frombuffer(fig.canvas.tostring_rgb(), dtype=np.uint8)
    data = data.reshape(fig.canvas.get_width_height()[::-1] + (3,))
    data = cv2.cvtColor(data, cv2.COLOR_RGB2BGR)
    bev_video_writer.write(data)
    cv2.imshow('result', data)
    cv2.waitKey(1)

    front_img = cv2.imdecode(np.frombuffer(front_cam.data, np.uint8), cv2.IMREAD_COLOR)
    back_img = cv2.imdecode(np.frombuffer(back_cam.data, np.uint8), cv2.IMREAD_COLOR)
    right_img = cv2.imdecode(np.frombuffer(right_cam.data, np.uint8), cv2.IMREAD_COLOR)
    left_img = cv2.imdecode(np.frombuffer(left_cam.data, np.uint8), cv2.IMREAD_COLOR)

    imgs = {
        "front": front_img,
        "back": back_img,
        "right": right_img,
        "left": left_img
    }

    for direction in ["front", "back", "right", "left"]:
         img = imgs[direction]
         cv2.putText(img, direction, (30, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 255), 2, cv2.LINE_AA)

    if det_res.num_boxes > 0:
        msg = det_res
        box_array = []
        for i in range(msg.num_boxes):
            print("score: ", msg.box3d_array[i].score)
            # if msg.box3d_array[i].score < 0.3:
            #     continue
            box = [msg.box3d_array[i].center_x, msg.box3d_array[i].center_y, msg.box3d_array[i].center_z,
                     msg.box3d_array[i].width, msg.box3d_array[i].length, msg.box3d_array[i].height,
                     msg.box3d_array[i].heading, msg.box3d_array[i].score]
            box_array.append(box)

        if len(box_array) > 0:
            box_array = np.array(box_array).reshape(-1, 8)
            for direction in ["front", "back", "right", "left"]:
                img = imgs[direction]

                cam_model = cam_models[direction]
                
                box = cam_model.world2cam(box_array[:, :3].T).T
                depth = box[:, 0]
                box_cam = box_array[depth > 0.05]
                corners = calculate_corners(box_cam).reshape(-1, 3)
                corners = cam_model.world2cam(corners.T)
                corners[0][corners[0] < 0.05] = 0.05
                corners = corners.T.reshape(-1, 8, 3)
                bboxes = []
                scores = box_array[:, 7]
                for corner, score in zip(corners, scores):
                    pixel_uv = cam_model.cam2image(corner.T).T
                    bboxes.append(pixel_uv)
                    score_txt = "{:.2f}".format(score)
                    loc_x, loc_y = int(min(pixel_uv[:,0])), int(max(min(pixel_uv[:,1]) - 5, 0))
                    if loc_x < 20:
                        loc_x = 20
                    if loc_y < 20:
                        loc_y = 20
                    cv2.putText(img, score_txt, (loc_x, loc_y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2, cv2.LINE_AA)
                imgs[direction] = plot_rect3d_on_img(img, len(bboxes), bboxes, color=(0, 0, 255))
    img_array = np.concatenate([np.concatenate( [imgs["left"], imgs["front"]], axis=1),
                np.concatenate((imgs["right"], imgs["back"]), axis=1)], axis=0)   
    img_array = cv2.resize(img_array, (img_array.shape[1]//2, img_array.shape[0]//2))
    video_writer.write(img_array) 

    data = cv2.resize(data, (img_array.shape[0], img_array.shape[0]))
    combined = np.concatenate([img_array, data], axis=1)
    combined_video.write(combined)

if __name__ == '__main__':
    ros_node = rospy.init_node('draw_detection_result', anonymous=True)
    args = parse_args()
    rosbag_path = args.rosbag
    vehicle = args.vehicle
    
    if vehicle == "Hycan":
        det_res_topic = '/hycan_detection_results'
        track_res_topic = '/hycan_track_results'
        front_cam_topic = '/miivii_gmsl_ros/camera1/compressed'
        back_cam_topic = '/miivii_gmsl_ros/camera2/compressed'
        right_cam_topic = '/miivii_gmsl_ros/camera3/compressed'
        left_cam_topic = '/miivii_gmsl_ros/camera4/compressed'
        height = 720
        width = 1280
        rear_offset = 1.4259
        vehicle_length = 4.6
        vehicle_width = 1.9
    else:
        det_res_topic = '/rock_detection_results'
        track_res_topic = '/rock_track_results'
        front_cam_topic = '/driver/fisheye/front/compressed'
        back_cam_topic = '/driver/fisheye/back/compressed'
        right_cam_topic = '/driver/fisheye/right/compressed'
        left_cam_topic = '/driver/fisheye/left/compressed'
        height = 1080
        width = 1920
        rear_offset = 0
        vehicle_length = 4.5
        vehicle_width = 1.8

    cam_models = {
        "front": CamModel("front", vehicle),
        "back": CamModel("back",vehicle),
        "right": CamModel("right", vehicle),
        "left": CamModel("left", vehicle)
    }

    lidar_model = Lidar_transformation(vehicle)

    front_sub = Subscriber(front_cam_topic, CompressedImage)
    back_sub = Subscriber(back_cam_topic, CompressedImage)
    right_sub = Subscriber(right_cam_topic, CompressedImage)
    left_sub = Subscriber(left_cam_topic, CompressedImage)
    det_res_sub = Subscriber(det_res_topic, DetectionResults)
    track_res_sub = Subscriber(track_res_topic, DetectionResults)
    
    # sheng
    video_writer = cv2.VideoWriter('{}.avi'.format(rosbag_path.split('/')[-1].split('.bag')[0]), 
                                   cv2.VideoWriter_fourcc(*'XVID'), 
                                   10, 
                                   (width, height))
    bev_video_writer = cv2.VideoWriter('{}_bev.avi'.format(rosbag_path.split('/')[-1].split('.bag')[0]),
                                        cv2.VideoWriter_fourcc(*'XVID'),
                                        10,
                                        (1000, 1000))
    combined_video = cv2.VideoWriter('{}_combined.avi'.format(rosbag_path.split('/')[-1].split('.bag')[0]),
                                     cv2.VideoWriter_fourcc(*'XVID'),
                                     10,
                                     (width + height, height))
    ts = ApproximateTimeSynchronizer([front_sub, back_sub, right_sub, left_sub, det_res_sub, track_res_sub], queue_size=50, slop=0.1)

    ts.registerCallback(draw_boxes_callback)
    bag = rosbag.Bag(rosbag_path)
    for topic, msg, t in bag.read_messages(topics=[front_cam_topic, back_cam_topic, right_cam_topic, left_cam_topic, det_res_topic, track_res_topic]):
        if topic == front_cam_topic:
            front_sub.signalMessage(msg)
        elif topic == back_cam_topic:
            back_sub.signalMessage(msg)
        elif topic == right_cam_topic:
            right_sub.signalMessage(msg)
        elif topic == left_cam_topic:
            left_sub.signalMessage(msg)
        elif topic == det_res_topic:
            det_res_sub.signalMessage(msg)
        elif topic == track_res_topic:
            track_res_sub.signalMessage(msg)
    bag.close()
    video_writer.release()
    bev_video_writer.release()
    combined_video.release()