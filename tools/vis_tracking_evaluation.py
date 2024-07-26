import os
import json

import rosbag
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


def load_json(path: str):
    json_file_names = os.listdir(path)
    data = {}
    for name in json_file_names:
        if os.path.isdir(os.path.join(path, name)):
            continue
        if 'json' not in name:
            continue

        with open(os.path.join(path, name), 'r') as f:
            f_dict = json.load(f)
            data[name[:-5]] = f_dict
            print('loading {}'.format(name))
    return data

def load_scenario_json(json_path: str, scenario_list_path: str):
    scenario_list= pd.read_csv(scenario_list_path, converters={'start': str, 'end': str, 'type': str})
    data = load_json(json_path)
    normal_data = {}
    turn_data = {}
    uturn_data = {}
    for item in data:
        curr_t = float(item)
        curr_type = 0
        for _, scenario_row in scenario_list.iterrows():
            if float(scenario_row['start']) <= curr_t <= float(scenario_row['end']):
                curr_type = 1 if scenario_row['type'] == 'turn' else 2
                break
        if curr_type == 0:
            normal_data[item] = data[item]
        elif curr_type == 1:
            turn_data[item] = data[item]
        else:
            uturn_data[item] = data[item]
    print(len(normal_data), len(turn_data), len(uturn_data))
    return normal_data, turn_data, uturn_data
                    
    

def load_merge_metrics(metrics_path_list):
    metrics_list = []
    for path in metrics_path_list:
        metrics_path = os.path.join(path, 'metrics', 'metrics.csv')
        metrics_df = pd.read_csv(metrics_path)
        metrics_list.append(metrics_df)
    all_metrics_df = pd.concat(metrics_list, axis=0)
    return all_metrics_df
        


def calc_iou(box_1: tuple, box_2: tuple):
    # (x, y, w, h)
    x_intersec_1 = max(box_1[0], box_2[0])
    y_intersec_1 = max(box_1[1], box_2[1])
    x_intersec_2 = min(box_1[0] + box_1[2], box_2[0] + box_2[2])
    y_intersec_2 = min(box_1[1] + box_1[3], box_2[1] + box_2[3])
    w_intersec = max(x_intersec_2 - x_intersec_1, 0.0)
    h_intersec = max(y_intersec_2 - y_intersec_1, 0.0)
    area_intersec = w_intersec * h_intersec

    area_union = box_1[2] * box_1[3] + box_2[2] * box_2[3] - area_intersec

    iou = area_intersec / (area_union + 1e-6)
    return iou


def calc_center_dis(box_1: tuple, box_2: tuple):
    center_1 = np.array([box_1[0] + box_1[2] / 2, box_1[1] + box_1[3] / 2])
    center_2 = np.array([box_2[0] + box_2[2] / 2, box_2[1] + box_2[3] / 2])

    dis = np.linalg.norm(center_1 - center_2)
    return dis


def evaluation(pred_data, gt_data, pred_path):
    # left_valid_cnt = 0
    # right_valid_cnt = 0
    left_iou_list = []
    right_iou_list = []
    left_dis_list = []
    right_dis_list = []
    for pred_t in pred_data:
        if pred_t not in gt_data:
            continue
        pred_boxes = pred_data[pred_t]
        gt_boxes = gt_data[pred_t]
        # left box
        pred_left = pred_boxes['left_rearlight']
        gt_left = gt_boxes['left_rearlight']
        if pred_left is not None and gt_left is not None:
            pred_left = (pred_left['x'], pred_left['y'],
                         pred_left['w'], pred_left['h'])
            gt_left = (gt_left['x'], gt_left['y'], gt_left['w'], gt_left['h'])
            left_iou = calc_iou(pred_left, gt_left)
            left_dis = calc_center_dis(pred_left, gt_left)
            left_iou_list.append(left_iou)
            left_dis_list.append(left_dis)
        # right box
        pred_right = pred_boxes['right_rearlight']
        gt_right = gt_boxes['right_rearlight']
        if pred_right is not None and gt_right is not None:
            pred_right = (pred_right['x'], pred_right['y'],
                          pred_right['w'], pred_right['h'])
            gt_right = (gt_right['x'], gt_right['y'],
                        gt_right['w'], gt_right['h'])
            right_iou = calc_iou(pred_right, gt_right)
            right_dis = calc_center_dis(pred_right, gt_right)
            right_iou_list.append(right_iou)
            right_dis_list.append(right_dis)

    iou_list = np.array(left_iou_list + right_iou_list)
    dis_list = np.array(left_dis_list + right_dis_list)
    metrics = np.concatenate(
        [iou_list.reshape((-1, 1)), dis_list.reshape((-1, 1))], axis=1)
    metrics_df = pd.DataFrame(metrics, columns=['iou', 'dis'])
    if not os.path.exists(os.path.join(pred_path, 'metrics')):
        os.makedirs(os.path.join(pred_path, 'metrics'))
    metrics_df.to_csv(os.path.join(pred_path, 'metrics', 'metrics.csv'), index=False)

    return metrics_df


def evaluation_scenario(pred_path: str, gt_path: str, scenario_list_path: str):
    normal_pred_data, turn_pred_data, uturn_pred_data = load_scenario_json(pred_path, scenario_list_path)
    normal_gt_data, turn_gt_data, uturn_gt_data = load_scenario_json(gt_path, scenario_list_path)
    normal_metrics = evaluation(normal_pred_data, normal_gt_data)
    turn_metrics = evaluation(turn_pred_data, turn_gt_data)
    uturn_metrics = evaluation(uturn_pred_data, uturn_gt_data)
    
    if not os.path.exists(os.path.join(pred_path, 'metrics')):
        os.makedirs(os.path.join(pred_path, 'metrics'))
    normal_metrics.to_csv(os.path.join(pred_path, 'metrics', 'normal_metrics.csv'))
    turn_metrics.to_csv(os.path.join(pred_path, 'metrics', 'turn_metrics.csv'))
    uturn_metrics.to_csv(os.path.join(pred_path, 'metrics', 'uturn_metrics.csv'))
        
def analyze_method_metrics(metrics_path: str, name: str='metrics'):
    metrics_df = pd.read_csv(os.path.join(metrics_path, 'metrics', '{}.csv'.format(name)))
    metrics_df_valid = metrics_df[metrics_df['iou'] > -0.1]
    mean_iou = metrics_df_valid['iou'].mean()
    mean_dis = metrics_df_valid['dis'].mean()
    print('-------------------------------------------------')
    print('{} mean_iou'.format(name), mean_iou)
    print('{} mean_dis'.format(name), mean_dis)
    
    ap_iou_list = []
    print('AP-IOU')
    for iou_th in np.arange(0.0, 1.0, 0.05):
        iou_above_th = metrics_df_valid[metrics_df_valid['iou'] > iou_th]
        ap_iou = iou_above_th.shape[0] / metrics_df_valid.shape[0]
        print('{} AP-IOU'.format(iou_th), ap_iou)
        ap_iou_list.append([iou_th, ap_iou])
    ap_dis_list = []
    print('AP-DIS')
    for dis_th in np.arange(0.0, 105.0, 5.0):
        dis_below_th = metrics_df_valid[metrics_df_valid['dis'] < dis_th]
        ap_dis = dis_below_th.shape[0] / metrics_df_valid.shape[0]
        print('{} AP-DIS'.format(dis_th), ap_dis)
        ap_dis_list.append([dis_th, ap_dis])
    
    ap_iou_list = pd.DataFrame(np.array(ap_iou_list), columns=['threshold', 'ap_iou'])
    ap_dis_list = pd.DataFrame(np.array(ap_dis_list), columns=['threshold', 'ap_dis'])
    ap_iou_list.to_csv(os.path.join(metrics_path, 'metrics', 'ap_iou.csv'), index=False)
    ap_dis_list.to_csv(os.path.join(metrics_path, 'metrics', 'ap_dis.csv'), index=False)



def dump_bag_to_json(bag_path: str, save_path: str):
    bag = rosbag.Bag(bag_path, 'r')
    if not os.path.exists(save_path):
        os.makedirs(save_path)
    box_data = bag.read_messages('/tracking/components_tracking/rearlight')
    cnt = 0
    for _, msg, t in box_data:
        timestamp = msg.header.stamp.to_sec()
        data = {}
        data['vehicle'] = None
        left_box = msg.boxes[0]
        left_box_dict = {'x': float(left_box.center_y - left_box.width / 2),
                         'y': float(left_box.center_x - left_box.height / 2),
                         'w': float(left_box.width),
                         'h': float(left_box.height)}
        right_box = msg.boxes[1]
        right_box_dict = {'x': float(right_box.center_y - right_box.width / 2),
                          'y': float(right_box.center_x - right_box.height / 2),
                          'w': float(right_box.width),
                          'h': float(right_box.height)}
        data['left_rearlight'] = left_box_dict
        data['right_rearlight'] = right_box_dict
        
        with open(os.path.join(save_path, '{}.json'.format(timestamp)), 'w') as f:
            json.dump(data, f)
        cnt += 1
        print('{}th, save to {}'.format(cnt, os.path.join(save_path, '{}.json'.format(timestamp))))

def plot_success_curve(method_path_list, method_name_list, method_marker_list):
    assert len(method_path_list) == len(method_name_list), 'The lengths of method_path_list and method_name_list should be equal!'
    assert len(method_path_list) == len(method_marker_list), 'The lengths of method_path_list and method_marker_list should be equal!'
    ap_iou_list = []
    ap_dis_list = []
    for path in method_path_list:
        ap_iou = pd.read_csv(os.path.join(path, 'ap_iou.csv')).values
        ap_dis = pd.read_csv(os.path.join(path, 'ap_dis.csv')).values
        ap_iou_list.append(ap_iou)
        ap_dis_list.append(ap_dis)
    
    plt.style.use('seaborn')
    fig, ax = plt.subplots()
    for data, name, marker in zip(ap_iou_list, method_name_list, method_marker_list):
        ax.plot(data[:, 0], 100 * data[:, 1], label=name, marker=marker)
        ax.legend()
        ax.set_xlabel('Overlap threshold')
        ax.set_ylabel('Overlap precision (%)')
        ax.set_title('Success Plot based on IOU')
    plt.savefig('./iou_success_plot.png')

    fig, ax = plt.subplots()
    for data, name, marker in zip(ap_dis_list, method_name_list, method_marker_list):
        ax.plot(data[:, 0], 100 * data[:, 1], label=name, marker=marker)
        ax.legend()
        ax.set_xlabel('Distance threshold')
        ax.set_ylabel('Distance precision (%)')
        ax.set_title('Success Plot based on Distance')
    plt.savefig('./dis_success_plot.png')


if __name__ == '__main__':
    # # first, dump inference result bags info json files
    # save_path = '/mnt/d/platoon_dataset/2022_10_20/evaluation/2d/tomp/inference_result'
    # bag_path = '/mnt/d/platoon_dataset/2022_10_20/evaluation/2d/tomp/2023-03-20-19-24-23.bag'
    # dump_bag_to_json(bag_path, save_path)
    
    # # evaluate inference result of one method, then save iou and dis to csv
    # pred_path = '/mnt/d/platoon_dataset/2022_10_20/evaluation/2d/tomp/inference_result'
    # gt_path = '/mnt/d/platoon_dataset/2022_10_20/evaluation/gt/2d'
    # pred_data = load_json(pred_path)
    # gt_data = load_json(gt_path)
    # evaluation(pred_data, gt_data, pred_path)
    
    # # all scenarios metrics
    # analyze_method_metrics('/mnt/d/platoon_dataset/2022_10_20/evaluation/2d/our_method_long_term/inference_result')
    # analyze_method_metrics('/mnt/d/platoon_dataset/2022_10_20/evaluation/2d/rlt_dimp/inference_result')
    # analyze_method_metrics('/mnt/d/platoon_dataset/2022_10_20/evaluation/2d/keeptrack/inference_result')
    # analyze_method_metrics('/mnt/d/platoon_dataset/2022_10_20/evaluation/2d/tomp/inference_result')
    
    # plot success rate curve
    plot_success_curve(method_path_list=['/mnt/d/platoon_dataset/2022_10_20/evaluation/2d/our_method_long_term/inference_result/metrics', \
        '/mnt/d/platoon_dataset/2022_10_20/evaluation/2d/rlt_dimp_1/inference_result/metrics', \
            '/mnt/d/platoon_dataset/2022_10_20/evaluation/2d/keeptrack/inference_result/metrics', \
                '/mnt/d/platoon_dataset/2022_10_20/evaluation/2d/tomp/inference_result/metrics'], method_name_list=['Our method', 'RLT-DiMP', 'KeepTrack', 'ToMP'], \
            method_marker_list=['o', 'v', '^', 's'])
    
    # scenario matrics
    # evaluation_scenario('/mnt/d/platoon_dataset/2022_10_20/evaluation/2d/compare/inference_result', \
    #     '/mnt/d/platoon_dataset/2022_10_20/evaluation/gt/2d', \
    #     '/mnt/d/platoon_dataset/2022_10_20/evaluation/scenario.csv')
    # evaluation_scenario('/mnt/d/platoon_dataset/2022_10_20/evaluation/2d/compare/inference_result_cp', \
    #     '/mnt/d/platoon_dataset/2022_10_20/evaluation/gt/2d', \
    #     '/mnt/d/platoon_dataset/2022_10_20/evaluation/scenario.csv')
    
    # # calc scenario metrics
    # print('our method')
    # analyze_method_metrics('/mnt/d/platoon_dataset/2022_10_20/evaluation/2d/our_method/inference_result',\
    #     'metrics')
    # analyze_method_metrics('/mnt/d/platoon_dataset/2022_10_20/evaluation/2d/our_method/inference_result',\
    #     'normal_metrics')
    # analyze_method_metrics('/mnt/d/platoon_dataset/2022_10_20/evaluation/2d/our_method/inference_result',\
    #     'turn_metrics')
    # analyze_method_metrics('/mnt/d/platoon_dataset/2022_10_20/evaluation/2d/our_method/inference_result',\
    #     'uturn_metrics')
    # print('compare')
    # analyze_method_metrics('/mnt/d/platoon_dataset/2022_10_20/evaluation/2d/compare/inference_result_cp',\
    #     'metrics')
    # analyze_method_metrics('/mnt/d/platoon_dataset/2022_10_20/evaluation/2d/compare/inference_result_cp',\
    #     'normal_metrics')
    # analyze_method_metrics('/mnt/d/platoon_dataset/2022_10_20/evaluation/2d/compare/inference_result_cp',\
    #     'turn_metrics')
    # analyze_method_metrics('/mnt/d/platoon_dataset/2022_10_20/evaluation/2d/compare/inference_result_cp',\
    #     'uturn_metrics')
    
    
    # analyze by scenario
    # load_scenario_json('/mnt/d/platoon_dataset/2022_10_20/evaluation/gt/2d', '/mnt/d/platoon_dataset/2022_10_20/evaluation/scenario.csv')
    
    
    
    # plot_success_curve('/mnt/d/platoon_dataset/2022_10_20/evaluation/2d/Precision.csv')
