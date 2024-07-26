#!/usr/bin/env python
# -*- encoding: utf-8 -*-
'''
# AUTHOR: Haoran Wu
# FILE: vehicle_tracking_evaluation.py
# DATE: 2023/02/13 周一
# TIME: 21:28:27
'''



import os
import json
import time

import rosbag
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

from vis_tracking_visualization import generate_visual_tracking_seg
from vis_tracking_evaluation import dump_bag_to_json


def quart_to_rpy(x, y, z, w):
    roll  = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = np.arcsin(2 * (w * y - x * z))
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (z * z + y * y))
    
    return roll, pitch, yaw


def calc_err(pred: dict, gt: dict):
    # pred: {'x', 'y', 'yaw'}
    # gt: {'x', 'y', 'l', 'yaw'}
    pred_pos = np.array([pred['x'], pred['y']])
    gt_pos = np.array([gt['x'] + (-gt['l'] / 2 + 0.15) * np.cos(gt['yaw']), gt['y'] + (-gt['l'] / 2 + 0.15) * np.sin(gt['yaw'])])
    diff = pred_pos - gt_pos
    x_err = np.dot(diff, np.array([np.cos(gt['yaw']), np.sin(gt['yaw'])]))
    y_err = np.dot(diff, np.array([-np.sin(gt['yaw']), np.cos(gt['yaw'])]))
    yaw_err = pred['yaw'] - gt['yaw']
    
    return {'x_err': x_err, 'y_err': y_err, 'yaw_err': yaw_err}


def evaluation_bag(bag_path: str, json_path: str):
    """calculate pose tracking error and save to csv file
    Args:
        bag_path (str): path including a series of bag files
        json_path (str): path including pose tracking gt files
    """
    gt_name_list = os.listdir(json_path)
    gt_data = {}
    print('loading gt data...')
    for name in gt_name_list:
        if os.path.isdir(os.path.join(json_path, name)):
            continue
        if '.json' not in name:
            continue
        print('loading gt data {}...'.format(name))
        with open(os.path.join(json_path, name), 'r') as f:
            f_dict = json.load(f)
            if f_dict is not None and 'vehicle' in f_dict:
                if 'x' in f_dict['vehicle'] and f_dict['vehicle']['x'] is not None:
                    gt_data[name[:-5]] = f_dict['vehicle']

    print('parsing bag file...')
    bag_name_list = os.listdir(bag_path)
    count = 0
    pred_data = []
    for name in bag_name_list:
        if os.path.isdir(os.path.join(bag_path, name)):
            continue
        if '.bag' not in name:
            continue
        bag = rosbag.Bag(os.path.join(bag_path, name))
        box_data = bag.read_messages('/tracking/pose_lidar')
        for _, msg, _ in box_data:
            timestamp = msg.header.stamp.to_sec()
            if str(timestamp) not in gt_data:
                continue
            count += 1
            curr_box = {}
            curr_box['x'] = msg.pose.position.x
            curr_box['y'] = msg.pose.position.y
            _, _, yaw = quart_to_rpy(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)
            curr_box['yaw'] = yaw
            err = calc_err(curr_box, gt_data[str(timestamp)])
            err = [timestamp, err['x_err'], err['y_err'], err['yaw_err']]
            pred_data.append(err)
            print(len(pred_data))
            print('{}th, calc err of {}...'.format(count, timestamp))
            # time.sleep(0.1)
            
        bag.close()
    print('data volume: {}'.format(len(pred_data)))
    pred_data = pd.DataFrame(pred_data, columns=['timestamp', 'x_err', 'y_err', 'yaw_err'])
    if not os.path.exists(os.path.join(bag_path, 'metrics')):
        os.makedirs(os.path.join(bag_path, 'metrics'))
    pred_data.to_csv(os.path.join(bag_path, 'metrics', 'error.csv'))

def evaluation_one_bag(bag_path: str, gt_path):
    dir_name = os.path.split(bag_path)[0]
    scenario_name = os.path.splitext(os.path.split(bag_path)[1])[0]
    if os.path.exists(os.path.join(dir_name, 'metrics', '{}_error.csv'.format(scenario_name))):
        error_data = pd.read_csv(os.path.join(dir_name, 'metrics', '{}_error.csv'.format(scenario_name)),
                                 converters={'timestamp': str, 'x_err': float, 'y_err': float, 'yaw_err': float})
        return error_data
    
    gt_name_list = os.listdir(gt_path)
    gt_data = {}
    print('loading gt data...')
    for name in gt_name_list:
        if os.path.isdir(os.path.join(gt_path, name)):
            continue
        if '.json' not in name:
            continue
        print('loading gt data {}...'.format(name))
        with open(os.path.join(gt_path, name), 'r') as f:
            f_dict = json.load(f)
            if f_dict is not None and 'vehicle' in f_dict:
                if 'x' in f_dict['vehicle'] and f_dict['vehicle']['x'] is not None:
                    gt_data[name[:-5]] = f_dict['vehicle']
    
    print('parsing bag file...')
    count = 0
    error_data = []
    bag = rosbag.Bag(bag_path)
    box_data = bag.read_messages('/tracking/pose_lidar')
    
    curr_t = None
    curr_t_err = {'x_err': [], 'y_err': [], 'yaw_err': []}
    for _, msg, _ in box_data:
        timestamp = msg.header.stamp.to_sec()
        if str(timestamp) not in gt_data:
            continue

        if curr_t is None:
            curr_t = timestamp
        elif curr_t != timestamp:
            min_x_err = min(curr_t_err['x_err'], key=abs)
            min_y_err = min(curr_t_err['y_err'], key=abs)
            min_yaw_err = min(curr_t_err['yaw_err'], key=abs)
            error_data.append([curr_t, min_x_err, min_y_err, min_yaw_err])
            
            curr_t_err = {'x_err': [], 'y_err': [], 'yaw_err': []}
            curr_t = timestamp
            
        count += 1
        curr_box = {}
        curr_box['x'] = msg.pose.position.x
        curr_box['y'] = msg.pose.position.y
        _, _, yaw = quart_to_rpy(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)
        curr_box['yaw'] = yaw
        err = calc_err(curr_box, gt_data[str(timestamp)])
        curr_t_err['x_err'].append(err['x_err'])
        curr_t_err['y_err'].append(err['y_err'])
        curr_t_err['yaw_err'].append(err['yaw_err'])

        print('{}th, calc err of {}...'.format(count, timestamp))
    error_data = pd.DataFrame(error_data, columns=['timestamp', 'x_err', 'y_err', 'yaw_err'])
    
    if not os.path.exists(os.path.join(dir_name, 'metrics')):
        os.makedirs(os.path.join(dir_name, 'metrics'))
    error_data.to_csv(os.path.join(dir_name, 'metrics', '{}_error.csv'.format(scenario_name)), index=False)
    
    return error_data

def calc_rmse_mae_std(error):
    rmse = np.sqrt(np.mean(np.square(error)))
    mae = np.mean(np.abs(error))
    std = np.std(error)
    return rmse, mae, std

def evaluation_metrics(metrics_path: str=None, error: pd.DataFrame=None):
    if error is None:
        error = pd.read_csv(metrics_path)
    x_err = error['x_err'].values.flatten()
    x_err[x_err > 0] = np.clip(x_err[x_err > 0] - 0.1, 0.0, 100)
    x_err[x_err < 0] = np.clip(x_err[x_err < 0] + 0.1, -100.0, 0.0)
    y_err = error['y_err'].values.flatten()
    yaw_err = error['yaw_err'].values.flatten()
    
    x_rmse, x_mae, x_err_std = calc_rmse_mae_std(x_err)
    y_rmse, y_mae, y_err_std = calc_rmse_mae_std(y_err)
    yaw_rmse, yaw_mae, yaw_err_std = calc_rmse_mae_std(yaw_err)
    
    print('x_rmse: {}, x_mae: {}, x_err_std: {}'.format(x_rmse, x_mae, x_err_std))
    print('y_rmse: {}, y_mae: {}, y_err_std: {}'.format(y_rmse, y_mae, y_err_std))
    print('yaw_rmse: {}, yaw_mae: {}, yaw_err_std: {}'.format(yaw_rmse, yaw_mae, yaw_err_std))
    
    # x_err = x_err[np.abs(x_err) < 1.0]
    # y_err = y_err[np.abs(y_err) < 1.0]
    # plt.boxplot([x_err, y_err], labels=['x', 'y'])
    # plt.savefig('./test.png')

def evaluation_scenario(metrics_path: str, scenario_list_path: str):
    scenario_list= pd.read_csv(scenario_list_path, converters={'start': str, 'end': str, 'type': str})
    error = pd.read_csv(metrics_path, converters={'timestamp': str, 'x_err': float, 'y_err': float, 'yaw_err': float})
    
    normal_data = []
    turn_data = []
    uturn_data = []
    for _, err_row in error.iterrows():
        curr_t = float(err_row['timestamp'])
        curr_type = 0
        for _, scenario_row in scenario_list.iterrows():
            if float(scenario_row['start']) <= curr_t <= float(scenario_row['end']):
                curr_type = 1 if scenario_row['type'] == 'turn' else 2
                break
        if curr_type == 0:
            normal_data.append([err_row['x_err'], err_row['y_err'], err_row['yaw_err']])
        elif curr_type == 1:
            turn_data.append([err_row['x_err'], err_row['y_err'], err_row['yaw_err']])
        elif curr_type == 2:
            uturn_data.append([err_row['x_err'], err_row['y_err'], err_row['yaw_err']])
    print(len(normal_data), len(turn_data), len(uturn_data))
    normal_data = pd.DataFrame(normal_data, columns=['x_err', 'y_err', 'yaw_err'])
    turn_data = pd.DataFrame(turn_data, columns=['x_err', 'y_err', 'yaw_err'])
    uturn_data = pd.DataFrame(uturn_data, columns=['x_err', 'y_err', 'yaw_err'])
    
    print('scenario normal:')
    evaluation_metrics(None, normal_data)
    print('scenario turn:')
    evaluation_metrics(None, turn_data)
    print('scenario uturn:')
    evaluation_metrics(None, uturn_data)

def v2v_ablation(v2v_path: str, no_v2v_path: str):
    v2v_error = pd.read_csv(v2v_path, converters={'timestamp': str, 'x_err': float, 'y_err': float, 'yaw_err': float})
    no_v2v_error = pd.read_csv(no_v2v_path, converters={'timestamp': str, 'x_err': float, 'y_err': float, 'yaw_err': float})
    min_t = np.min([float(v2v_error.iloc[0]['timestamp']), float(no_v2v_error.iloc[0]['timestamp'])])
    v2v_y_err = np.abs(v2v_error['y_err'].values.flatten())
    no_v2v_y_err = np.abs(no_v2v_error['y_err'].values.flatten())
    v2v_t = v2v_error['timestamp'].values.flatten().astype(np.float64) - min_t
    no_v2v_t = no_v2v_error['timestamp'].values.flatten().astype(np.float64) - min_t
    
    plt.style.use('seaborn')
    fig, ax = plt.subplots()
    ax.plot(v2v_t, v2v_y_err, label='V2V')
    ax.plot(no_v2v_t, no_v2v_y_err, label='no V2V')
    ax.set_title('Lateral Position Absolute Error')
    ax.set_xlabel('t(s)')
    ax.set_ylabel('absolute error(m)')
    ax.legend()
    plt.savefig('./test.png')

def evaluation_ablation_study(normal_bag_path: str, ablation_bag_path: str, gt_path: str, normal_label: str, ablation_label: str, 
                              t_start: float=None, t_end: float=None):
    normal_error_data = evaluation_one_bag(normal_bag_path, gt_path)
    ablation_error_data = evaluation_one_bag(ablation_bag_path, gt_path)
    if t_start is not None:
        min_t = t_start
    else:
        min_t = np.min([float(normal_error_data.iloc[0]['timestamp']), float(ablation_error_data.iloc[0]['timestamp'])])
    
    print('min t: {}'.format(min_t))
    normal_error_data['timestamp'] = normal_error_data['timestamp'].values.astype(float)
    ablation_error_data['timestamp'] = ablation_error_data['timestamp'].values.astype(float)
    normal_error_data = normal_error_data[normal_error_data['timestamp'] >= min_t]
    ablation_error_data = ablation_error_data[ablation_error_data['timestamp'] >= min_t]
    if t_end is not None:
        normal_error_data = normal_error_data[normal_error_data['timestamp'] <= t_end]
        ablation_error_data = ablation_error_data[ablation_error_data['timestamp'] <= t_end]
    
    normal_x_abs_err = np.abs(normal_error_data['x_err'])
    normal_y_abs_err = np.abs(normal_error_data['y_err'])
    normal_t = normal_error_data['timestamp'] - min_t
    ablation_x_abs_err = np.abs(ablation_error_data['x_err'])
    ablation_y_abs_err = np.abs(ablation_error_data['y_err'])
    ablation_t = ablation_error_data['timestamp'] - min_t
    
    plt.style.use('seaborn')
    fig, ax = plt.subplots()
    ax.plot(normal_t, normal_x_abs_err, label=normal_label)
    ax.plot(ablation_t, ablation_x_abs_err, label=ablation_label)
    ax.set_title('Longitudinal Position Absolute Error')
    ax.set_xlabel('t(s)')
    ax.set_ylabel('absolute error(m)')
    ax.legend()
    plt.savefig('./test_x.png')
    
    fig, ax = plt.subplots()
    ax.plot(normal_t, normal_y_abs_err, label=normal_label)
    ax.plot(ablation_t, ablation_y_abs_err, label=ablation_label)
    ax.set_title('Lateral Position Absolute Error')
    ax.set_xlabel('t(s)')
    ax.set_ylabel('absolute error(m)')
    ax.legend()
    plt.savefig('./test_y.png')
    
    
if __name__ == '__main__':
    # # calculate error and save to csv file
    # evaluation_bag(bag_path='/mnt/d/platoon_dataset/2022_10_20/evaluation/3d/our_method_1', \
    #     json_path='/mnt/d/platoon_dataset/2022_10_20/evaluation/gt/3d')
    
    # vis('/mnt/d/platoon_dataset/2022_10_20/evaluation/3d/combine_cp/metrics/error.csv')
    
    # # evaluate all evaluation
    # evaluation_metrics('/mnt/d/platoon_dataset/2022_10_20/evaluation/3d/combine_cp/metrics/error.csv')
    
    # # evaluation scenario
    # evaluation_scenario('/mnt/d/platoon_dataset/2022_10_20/evaluation/3d/combine_cp/metrics/error.csv',\
    #     '/mnt/d/platoon_dataset/2022_10_20/evaluation/scenario.csv')
    
    ################################################ablation study############################################################
    # # consequent turn, rearlight ablation study
    # evaluation_ablation_study(normal_bag_path='/mnt/d/platoon_dataset/2022_10_20/evaluation/3d/ablation_study_cp/consequent_turn.bag', \
    #     ablation_bag_path='/mnt/d/platoon_dataset/2022_10_20/evaluation/3d/ablation_study_cp/consequent_turn_no_rearlight.bag', \
    #     gt_path='/mnt/d/platoon_dataset/2022_10_20/evaluation/gt/3d', \
    #     normal_label='normal', \
    #     ablation_label='no rearlight', \
    #     t_start=None, \
    #     t_end=1666253867.299093 + 30.0)
    
    # # normal scenario, radar ablation study
    # evaluation_ablation_study(normal_bag_path='/mnt/d/platoon_dataset/2022_10_20/evaluation/3d/ablation_study_cp/normal_1.bag', \
    #     ablation_bag_path='/mnt/d/platoon_dataset/2022_10_20/evaluation/3d/ablation_study_cp/normal_1_no_radar.bag', \
    #     gt_path='/mnt/d/platoon_dataset/2022_10_20/evaluation/gt/3d', \
    #     normal_label='normal', \
    #     ablation_label='no radar', \
    #     t_start=1666253350.09489, \
    #     t_end=1666253350.09489 + 90)
    
    # # strong light 1, radar ablation study
    # # no gt data
    # evaluation_ablation_study(normal_bag_path='/mnt/d/platoon_dataset/2022_10_20/evaluation/3d/ablation_study_cp/strong_light_1.bag', \
    #     ablation_bag_path='/mnt/d/platoon_dataset/2022_10_20/evaluation/3d/ablation_study_cp/strong_light_1_no_radar.bag', \
    #     gt_path='/mnt/d/platoon_dataset/2022_10_20/evaluation/gt/3d', \
    #     normal_label='normal', \
    #     ablation_label='no radar', \
    #     t_start=None, \
    #     t_end=None)
    
    # # strong light 2, radar ablation study
    # # no gt
    # evaluation_ablation_study(normal_bag_path='/mnt/d/platoon_dataset/2022_10_20/evaluation/3d/ablation_study_cp/strong_light_2.bag', \
    #     ablation_bag_path='/mnt/d/platoon_dataset/2022_10_20/evaluation/3d/ablation_study_cp/strong_light_2_no_radar.bag', \
    #     gt_path='/mnt/d/platoon_dataset/2022_10_20/evaluation/gt/3d', \
    #     normal_label='normal', \
    #     ablation_label='no radar', \
    #     t_start=None, \
    #     t_end=None)
    
    # # turn 1, rearlight ablation study
    # evaluation_ablation_study(normal_bag_path='/mnt/d/platoon_dataset/2022_10_20/evaluation/3d/ablation_study_cp/turn_1.bag', \
    #     ablation_bag_path='/mnt/d/platoon_dataset/2022_10_20/evaluation/3d/ablation_study_cp/turn_1_no_rearlight.bag', \
    #     gt_path='/mnt/d/platoon_dataset/2022_10_20/evaluation/gt/3d', \
    #     normal_label='normal', \
    #     ablation_label='no rearlight', \
    #     t_start=1666253361.100893, \
    #     t_end=1666253361.100893 + 15)
    
    # # uturn 1, rearlight ablation study
    # evaluation_ablation_study(normal_bag_path='/mnt/d/platoon_dataset/2022_10_20/evaluation/3d/ablation_study_cp/uturn_1.bag', \
    #     ablation_bag_path='/mnt/d/platoon_dataset/2022_10_20/evaluation/3d/ablation_study_cp/uturn_1_no_rearlight.bag', \
    #     gt_path='/mnt/d/platoon_dataset/2022_10_20/evaluation/gt/3d', \
    #     normal_label='normal', \
    #     ablation_label='no rearlight', \
    #     t_start=1666253484.102698 + 2, \
    #     t_end=1666253484.102698 + 25)
    
    # # uturn 2, v2v ablation study
    # evaluation_ablation_study(normal_bag_path='/mnt/d/platoon_dataset/2022_10_20/evaluation/3d/ablation_study_cp/uturn_2.bag', \
    #     ablation_bag_path='/mnt/d/platoon_dataset/2022_10_20/evaluation/3d/ablation_study_cp/uturn_2_no_v2v.bag', \
    #     gt_path='/mnt/d/platoon_dataset/2022_10_20/evaluation/gt/3d', \
    #     normal_label='normal', \
    #     ablation_label='no V2V', \
    #     t_start=None, \
    #     t_end=1666253472.096261 + 35)
    
    ################################################visual tracking visualization############################################################
    # # dump bag to json files
    # dump_bag_to_json('/mnt/d/platoon_dataset/2022_10_20/evaluation/3d/ablation_study_cp/uturn_2.bag',
    #                  '/mnt/d/platoon_dataset/2022_10_20/evaluation/3d/ablation_study_cp/inference_result/2d')
    
    ## plot rearlight on image, and save to video file
    # # consequent turn
    # generate_visual_tracking_seg(raw_data_path='/mnt/d/platoon_dataset/2022_10_20/2022-10-20-16-15-59.bag',
    #                              video_save_path='/mnt/d/platoon_dataset/2022_10_20/evaluation/3d/ablation_study_cp/visualization/2d/consequent_turn.mp4',
    #                              inference_path='/mnt/d/platoon_dataset/2022_10_20/evaluation/3d/ablation_study_cp/inference_result/2d',
    #                              t_start=1666253867.30,
    #                              t_end=1666253867.299093 + 30.0)
    
    pass