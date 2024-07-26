import os
import json
from collections import deque

import rosbag
import rospy
import numpy as np
import pandas as pd
import cv2
# import matplotlib.pyplot as plt

from vis_tracking_evaluation import load_json

COLORS = [
    (255, 0, 0),
    (0, 255, 0,),
    (0, 0, 255),
    (0, 128, 255),
    (0, 255, 255),
    (128, 255, 0),
    (255, 255, 0),
    (255, 128, 0)
]

def plot_box_on_image(raw_data_path: str, video_save_path: str, inference_path_list: list, inference_name_list: list, inference_color_list: list, thickness: int=2):
    assert len(inference_path_list) == len(inference_name_list)
    inference_data_list = [load_json(path) for path in inference_path_list]
    
    bag = rosbag.Bag(raw_data_path, 'r')
    image_data = bag.read_messages('/driver/fisheye/front/compressed')
    
    # video writer
    out = cv2.VideoWriter(video_save_path, cv2.VideoWriter_fourcc('m', 'p', '4', 'v'), 10, (1920, 1080), True)
    not_found_cnt = 0
    left_pos_buffer = deque(maxlen=20)
    right_pos_buffer = deque(maxlen=20)
    for _, msg, t in image_data:
        print('load image {}'.format(t.to_sec()))
        # load image
        np_arr = np.frombuffer(msg.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        
        inference_timestamp = str(msg.header.stamp.to_sec())
        found_flag = False
        for i, (method_data, method_name, method_color) in \
            enumerate(zip(inference_data_list, inference_name_list, inference_color_list)): # for each method
            left_box = right_box = None
            if inference_timestamp in method_data:
                left_box = method_data[inference_timestamp]['left_rearlight']
                right_box = method_data[inference_timestamp]['right_rearlight']
            # elif last_boxes[i] is not None:
            #     left_box = last_boxes[i]['left_rearlight']
            #     right_box = last_boxes[i]['right_rearlight']
            if left_box is not None:
                print('found left box of method {}'.format(method_name))
                image = cv2.rectangle(image, \
                    (int(left_box['x']), int(left_box['y'])), \
                        (int(left_box['x'] + left_box['w']), int(left_box['y'] + left_box['h'])), \
                            color=COLORS[method_color], thickness=thickness)
                text_loc = (int(left_box['x']), int(left_box['y'])) if method_name == 'Ours' else (int(left_box['x'] + left_box['w']), int(left_box['y']))
                image = cv2.putText(image, 'left', text_loc, cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.6, color=COLORS[method_color], thickness=2)
                # # trajectory
                # left_pos_buffer.append((int(left_box['x'] + left_box['w'] / 2), int(left_box['y'] + left_box['h'] / 2)))
                # for j in range(len(left_pos_buffer) - 1):
                #     image = cv2.line(image, left_pos_buffer[j], left_pos_buffer[j + 1], color=COLORS[method_color], thickness=2)
                found_flag = True
            if right_box is not None:
                print('found right box of method {}'.format(method_name))
                image = cv2.rectangle(image, \
                    (int(right_box['x']), int(right_box['y'])), \
                        (int(right_box['x'] + right_box['w']), int(right_box['y'] + right_box['h'])), \
                            color=COLORS[method_color], thickness=thickness)
                text_loc = (int(right_box['x']), int(right_box['y'])) if method_name == 'Ours' else (int(right_box['x'] + right_box['w']), int(right_box['y']))
                image = cv2.putText(image, 'right', text_loc, cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.6, color=COLORS[method_color], thickness=2)
                # # trajectory
                # right_pos_buffer.append((int(right_box['x'] + right_box['w'] / 2), int(right_box['y'] + right_box['h'] / 2)))
                # for j in range(len(right_pos_buffer) - 1):
                #     image = cv2.line(image, right_pos_buffer[j], right_pos_buffer[j + 1], color=COLORS[method_color], thickness=2)
                found_flag = True
            # text
            image = cv2.putText(image, method_name, (1700, 100 + i * 40), cv2.FONT_HERSHEY_SIMPLEX, fontScale=1, color=COLORS[method_color], thickness=3)
        if found_flag or not_found_cnt == 3:
            out.write(image)
            not_found_cnt = 0
        else:
            not_found_cnt += 1
    out.release()
    bag.close()

def generate_visual_tracking_seg(raw_data_path: str, 
                                 video_save_path: str, 
                                 inference_path: str, 
                                 t_start: float=None, 
                                 t_end: float=None, 
                                 color: int=1, 
                                 thickness: int=2):
    inference_data = load_json(inference_path)
    
    bag = rosbag.Bag(raw_data_path, 'r')
    image_data = bag.read_messages('/driver/fisheye/front/compressed', 
                                   start_time=rospy.Time.from_sec(t_start) if t_start is not None else None,
                                   end_time=rospy.Time.from_sec(t_end) if t_end is not None else None)
    
    # video writer
    out = cv2.VideoWriter(video_save_path, cv2.VideoWriter_fourcc('m', 'p', '4', 'v'), 10, (1920, 1080), True)
    
    last_left_box = last_right_box = None
    for _, msg, t in image_data:
        print('load image {}'.format(t.to_sec()))
        # load image
        np_arr = np.frombuffer(msg.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        
        inference_timestamp = str(msg.header.stamp.to_sec())
        left_box = right_box = None
        if inference_timestamp in inference_data:
            left_box = inference_data[inference_timestamp]['left_rearlight']
            right_box = inference_data[inference_timestamp]['right_rearlight']
        
        if left_box is None:
            left_box = last_left_box
        if right_box is None:
            right_box = last_right_box

        if left_box is not None:
            print('found left box')
            image = cv2.rectangle(image, \
                (int(left_box['x']), int(left_box['y'])), \
                    (int(left_box['x'] + left_box['w']), int(left_box['y'] + left_box['h'])), \
                        color=COLORS[color], thickness=thickness)
            text_loc = (int(left_box['x']), int(left_box['y']))
            image = cv2.putText(image, 'left', text_loc, cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.6, color=COLORS[color], thickness=2)
            # # trajectory
            # left_pos_buffer.append((int(left_box['x'] + left_box['w'] / 2), int(left_box['y'] + left_box['h'] / 2)))
            # for j in range(len(left_pos_buffer) - 1):
            #     image = cv2.line(image, left_pos_buffer[j], left_pos_buffer[j + 1], color=COLORS[method_color], thickness=2)
        if right_box is not None:
            print('found right box')
            image = cv2.rectangle(image, \
                (int(right_box['x']), int(right_box['y'])), \
                    (int(right_box['x'] + right_box['w']), int(right_box['y'] + right_box['h'])), \
                        color=COLORS[color], thickness=thickness)
            text_loc = (int(right_box['x']), int(right_box['y']))
            image = cv2.putText(image, 'right', text_loc, cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.6, color=COLORS[color], thickness=2)
            # # trajectory
            # right_pos_buffer.append((int(right_box['x'] + right_box['w'] / 2), int(right_box['y'] + right_box['h'] / 2)))
            # for j in range(len(right_pos_buffer) - 1):
            #     image = cv2.line(image, right_pos_buffer[j], right_pos_buffer[j + 1], color=COLORS[method_color], thickness=2)
        out.write(image)
        last_left_box = left_box
        last_right_box = right_box

    out.release()
    bag.close()


if __name__ == '__main__':
    raw_data_path = '/mnt/d/platoon_dataset/2022_10_20/2022-10-20-16-36-56.bag' #
    video_save_path= '/mnt/d/platoon_dataset/2022_10_20/evaluation/2d/vis/2022-10-20-16-36-56.mp4'  #
    inference_path_list = ['/mnt/d/platoon_dataset/2022_10_20/evaluation/2d/our_method_long_term/inference_result', \
        '/mnt/d/platoon_dataset/2022_10_20/evaluation/2d/rlt_dimp/inference_result']
    inference_name_list = ['Ours', 'RLT-DiMP']
    inference_color_list = [1, 6]
    plot_box_on_image(raw_data_path, video_save_path, inference_path_list, inference_name_list, inference_color_list, thickness=2)
    # img = np.zeros((1080, 1920), np.uint8)
    # img = cv2.putText(img, 'Test', (1700, 100), cv2.FONT_HERSHEY_SIMPLEX, fontScale=1, color=(255, 255, 255), thickness=3)
    # img = cv2.putText(img, 'Test1', (1700, 140), cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.5, color=(255, 255, 255), thickness=3)
    # cv2.imwrite('./test.png', img)
