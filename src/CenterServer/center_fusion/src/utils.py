import numpy as np
from copy import deepcopy

def nms_depth(bbox_3d, vehicle_localization, angle_threshold, score_diff_threshold):
    """
        bbox_3d: [N, 8], x, y, z, w, l, h, yaw, score
        vehicle_localization: tuple, (x, y)
        angle_threshold: float, in radian
        score_diff_threshold: float
    """
    bbox_3d = deepcopy(bbox_3d)
    bbox_3d[:, :2] = bbox_3d[:, :2] - np.array(vehicle_localization)
    angle = np.arctan2(bbox_3d[:, 1], bbox_3d[:, 0])

    # sort by score
    sorted_idx = np.argsort(bbox_3d[:, 7])[::-1]
    bbox_3d = bbox_3d[sorted_idx]
    angle = angle[sorted_idx]

    # nms
    keep = []
    while len(bbox_3d) > 0:
        keep.append(sorted_idx[0])
        angle_diff = np.abs(angle - angle[0])
        angle_diff[angle_diff > np.pi] = 2 * np.pi - angle_diff[angle_diff > np.pi]
        print("angle_diff", angle_diff)
        if bbox_3d[0, 7] < score_diff_threshold + 0.05:
            score_diff = np.array([score_diff_threshold + 0.001] * len(bbox_3d))
        else:
            score_diff = bbox_3d[0, 7] - bbox_3d[:, 7]
        mask = ~((angle_diff < angle_threshold) * (score_diff > score_diff_threshold))
        mask[0] = False
        bbox_3d = bbox_3d[mask]
        angle = angle[mask]
        sorted_idx = sorted_idx[mask]

    return keep