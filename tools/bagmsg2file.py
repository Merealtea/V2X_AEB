#!/usr/bin/env python
# -*- encoding: utf-8 -*-
'''
# AUTHOR: Haoran Wu
# FILE: bagmsg2file.py
# DATE: 2022/10/21 周五
# TIME: 16:02:58
'''

import os
import glob

import argparse
import logging

import rosbag
import rospy
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import cv2
import open3d as o3d

logging.basicConfig(level=logging.INFO, format='[%(asctime)s] %(message)s')

parser = argparse.ArgumentParser()
parser.add_argument('--bag-path', type=str, required=True)
parser.add_argument('--save-path', type=str, required=True)
parser.add_argument('--fisheye', action='store_true', default=False)
parser.add_argument('--lidar', action='store_true', default=False)
parser.add_argument('--freq', type=int, default=2)
parser.add_argument('-s', '--start', type=float)
parser.add_argument('-e', '--end', type=float)


LIDAR_FREQ = 10
FISHEYE_FREQ = 30


def dump_image(bag_path: str, save_path: str, freq: int=10, start: float=None, end: float=None):
    """
    bag_path: path to ros bag
    save_path: path to save images
    freq: frequency to dump images
    start: when to start dumping, time(sec) from the beginning of bags
    end: when to end dumping, time(sec) from the beginning of bags
    """
    logging.info('-------------------------dumping image start-------------------------')
    bag = rosbag.Bag(bag_path, 'r')
    bag_start_time = bag.get_start_time()
    bag_end_time = bag.get_end_time()
    sampling_end_time = bag_end_time if end is None else min(bag_end_time, bag_start_time + end)
    
    if not os.path.exists(save_path):
        os.makedirs(save_path)
    
    image_data = bag.read_messages('/driver/fisheye/front/compressed')
    total = 1
    sampling_cycle_cnt = int(FISHEYE_FREQ / freq)
    count = -1
    for _, msg, t in image_data:
        log_str = 'timestamp: {}/{}'.format(t.to_sec(), sampling_end_time if sampling_end_time is not None else bag_end_time)
        if start is not None and t.to_sec() - bag_start_time < start:
            log_str += ', skip'
            logging.info(log_str)
            continue
        if end is not None and t.to_sec() - bag_start_time > end:
            break
        
        count = (count + 1) % sampling_cycle_cnt
        if count > 0:
            log_str += ', skip'
            logging.info(log_str)
            continue
        
        np_arr = np.frombuffer(msg.data, np.uint8)
        curr_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        # use bag time, not header stamp
        # timestamp = msg.header.stamp.to_sec()
        cv2.imwrite(os.path.join(save_path, '{}.jpg'.format(t.to_sec())), curr_img)
        log_str += ', dump {}th image'.format(total)
        logging.info(log_str)
        total += 1
    bag.close()
    logging.info('-------------------------dumping image done-------------------------')
    

def dump_lidar_pc(bag_path: str, save_path: str, freq: int=10, start: float=None, end: float=None):
    """
    bag_path: path to ros bag
    save_path: path to save images
    freq: frequency to dump pointcloud
    start: when to start dumping, time(sec) from the beginning of bags
    end: when to end dumping, time(sec) from the beginning of bags
    """
    logging.info('-------------------------dumping pointcloud start-------------------------')
    bag = rosbag.Bag(bag_path, 'r')
    bag_start_time = bag.get_start_time()
    bag_end_time = bag.get_end_time()
    sampling_end_time = bag_end_time if end is None else min(bag_end_time, bag_start_time + end)
    
    if not os.path.exists(save_path):
        os.makedirs(save_path)
    
    pc_data = bag.read_messages('/driver/hesai/pandar')
    total = 1
    sampling_cycle_cnt = int(LIDAR_FREQ / freq)
    count = -1
    for _, msg, t in pc_data:
        log_str = 'timestamp: {}/{}'.format(t.to_sec(), sampling_end_time if sampling_end_time is not None else bag_end_time)
        if start is not None and t.to_sec() - bag_start_time < start:
            log_str += ', skip'
            logging.info(log_str)
            continue
        if end is not None and t.to_sec() - bag_start_time > end:
            break
        
        count = (count + 1) % sampling_cycle_cnt
        if count > 0:
            log_str += ', skip'
            logging.info(log_str)
            continue
        
        pc_data = pc2.read_points(msg)
        pc_data = np.array(list(pc_data), dtype=np.float32)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pc_data[:, :3])
        o3d.io.write_point_cloud(os.path.join(save_path, '{}.pcd'.format(t.to_sec())), pcd, write_ascii=True)
        log_str += ', dump {}th pointcloud'.format(total)
        logging.info(log_str)
        total += 1
    bag.close()
    logging.info('-------------------------dumping pointcloud done-------------------------')

    
if __name__ == '__main__':
    args = parser.parse_args()
    
    if args.fisheye:
        image_save_path = os.path.join(args.save_path, 'image')
        if not os.path.exists(image_save_path):
            os.makedirs(image_save_path)
        dump_image(bag_path=args.bag_path, save_path=image_save_path, freq=args.freq, start=args.start, end=args.end)
    if args.lidar:
        pcd_save_path = os.path.join(args.save_path, 'pcd')
        if not os.path.exists(pcd_save_path):
            os.makedirs(pcd_save_path)
        dump_lidar_pc(bag_path=args.bag_path, save_path=pcd_save_path, freq=args.freq, start=args.start, end=args.end)
    
