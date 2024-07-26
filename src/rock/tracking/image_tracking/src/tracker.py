import sys
import time
import copy
sys.path.insert(0, './YOLOX')
from YOLOX.yolox.data.datasets.coco_classes import COCO_CLASSES
from detector import Detector
from deep_sort.utils.parser import get_config
from deep_sort.deep_sort import DeepSort
import torch
import cv2
from utils.visualize import vis_track, vis
import numpy as np


class_names = COCO_CLASSES

class Tracker():
    def __init__(self, filter_class=None, model='yolox-s', ckpt='yolox_s.pth.tar', trt=False):
        self.detector = Detector(model, ckpt, trt=trt)
        cfg = get_config()
        cfg.merge_from_file("deep_sort/configs/deep_sort.yaml")
        self.deepsort = DeepSort(cfg.DEEPSORT.REID_CKPT,
                            max_dist=cfg.DEEPSORT.MAX_DIST, min_confidence=cfg.DEEPSORT.MIN_CONFIDENCE,
                            nms_max_overlap=cfg.DEEPSORT.NMS_MAX_OVERLAP, max_iou_distance=cfg.DEEPSORT.MAX_IOU_DISTANCE,
                            max_age=cfg.DEEPSORT.MAX_AGE, n_init=cfg.DEEPSORT.N_INIT, nn_budget=cfg.DEEPSORT.NN_BUDGET,
                            use_cuda=True,
                            trt=trt)
        self.filter_class = filter_class
        self.trt = trt

    def update(self, image, filter_id=None):
        s = time.time()
        info = self.detector.detect(image, visual=False)
        e = time.time()
        outputs = []
        if info['box_nums']>0:
            bbox_xywh = []
            bbox_xyxy = []
            scores = []
            result_class = []
            cls_ids = []
            #bbox_xywh = torch.zeros((info['box_nums'], 4))
            for (x1, y1, x2, y2), class_id, score  in zip(info['boxes'],info['class_ids'],info['scores']):
                if self.filter_class and class_names[int(class_id)] not in self.filter_class:
                    continue
                bbox_xywh.append([int((x1+x2)/2), int((y1+y2)/2), x2-x1, y2-y1])
                bbox_xyxy.append([int(x1), int(y1), int(x2), int(y2)])
                scores.append(score)
                cls_ids.append(class_id)
                result_class.append(class_names[int(class_id)])
            bbox_xywh = torch.Tensor(bbox_xywh)
            outputs = self.deepsort.update(bbox_xywh, scores, image, filter_id=filter_id)

            # for vis detection res
            raw_img = copy.deepcopy(image)
            image = vis_track(image, outputs)
        
            bbox_xyxy = np.array(bbox_xyxy)
            detection_vis = vis(raw_img, bbox_xyxy, scores, cls_ids, 0.1)
            cv2.imshow('det', detection_vis)
            cv2.waitKey(10)



        return image, outputs
