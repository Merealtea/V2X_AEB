import numpy as np
import torch

from .deep.feature_extractor import Extractor
from .sort.nn_matching import NearestNeighborDistanceMetric
from .sort.preprocessing import non_max_suppression
from .sort.detection import Detection
from .sort.tracker import Tracker
import time

__all__ = ['DeepSort']


class DeepSort(object):
    def __init__(self, model_path, max_dist=0.2, min_confidence=0.3, nms_max_overlap=1.0, max_iou_distance=0.7, max_age=70, n_init=3, nn_budget=100, use_cuda=True, trt=False):
        self.min_confidence = min_confidence
        self.nms_max_overlap = nms_max_overlap

        self.extractor = Extractor(model_path, use_cuda=use_cuda, trt=trt)

        max_cosine_distance = max_dist
        nn_budget = 100
        metric = NearestNeighborDistanceMetric("cosine", max_cosine_distance, nn_budget)
        self.tracker = Tracker(metric, max_iou_distance=max_iou_distance, max_age=max_age, n_init=n_init)

    def update(self, bbox_xywh, confidences, ori_img, filter_id=None):
        self.tracker.filter(filter_id)

        self.height, self.width = ori_img.shape[:2]

        # filter detections
        s = time.time()
        curr_tracks = self.tracker.get_tracks()
        if len(curr_tracks) == 1:
            bbox_xyxy = []
            for xywh in bbox_xywh:
                x1, y1, x2, y2 = self._xywh_to_xyxy(xywh)
                xyxy_tensor = torch.Tensor([x1, y1, x2, y2]).unsqueeze(0)
                bbox_xyxy.append(xyxy_tensor)
            bbox_xyxy = torch.cat(bbox_xyxy, dim=0)
            track_xyxy = curr_tracks[0].to_tlbr()
            iou_with_track = torch.Tensor([self._get_IOU(track_xyxy, xyxy) for xyxy in bbox_xyxy])
            if torch.any(iou_with_track > 0.0):
                try:
                    selected_bbox = torch.topk(iou_with_track, 2).indices
                except:
                    try:
                        selected_bbox = torch.topk(iou_with_track, 1).indices
                    except:
                        selected_bbox = torch.topk(iou_with_track, 0).indices
            else:
                track_xywh = torch.Tensor([(track_xyxy[0] + track_xyxy[2]) / 2, (track_xyxy[1] + track_xyxy[3]) / 2,
                track_xyxy[2] - track_xyxy[0], track_xyxy[3] - track_xyxy[1]])
                dis = list(map(lambda xywh: torch.norm(xywh[:2] - track_xywh[:2], p=2), bbox_xywh.clone()))
                dis = torch.as_tensor(dis)
                try:
                    selected_bbox = torch.topk(dis, 2, largest=False).indices
                except:
                    try:
                        selected_bbox = torch.topk(dis, 1, largest=False).indices
                    except:
                        selected_bbox = torch.topk(dis, 0, largest=False).indices
                        
            bbox_xywh = bbox_xywh[selected_bbox]
            confidences = torch.as_tensor(confidences)
            confidences = list(confidences[selected_bbox])
        e = time.time()
        # print('filter detec time: {} sec'.format(e - s))
        # print('detc num: {}'.format(bbox_xywh.shape[0]))
        # generate detections
        s = time.time()
        features = self._get_features(bbox_xywh, ori_img)
        bbox_tlwh = self._xywh_to_tlwh(bbox_xywh)
        detections = [Detection(bbox_tlwh[i], conf, features[i]) for i,conf in enumerate(confidences) if conf>self.min_confidence]
        e = time.time()
        # print('feat time: {} sec'.format(e - s))

        # run on non-maximum supression
        s = time.time()
        boxes = np.array([d.tlwh for d in detections])
        scores = np.array([d.confidence for d in detections])
        indices = non_max_suppression(boxes, self.nms_max_overlap, scores)
        detections = [detections[i] for i in indices]
        e = time.time()
        # print('nms time: {} sec'.format(e - s))

        # update tracker
        s = time.time()
        self.tracker.predict()
        self.tracker.update(detections)
        e = time.time()
        # print('predict-update time: {} sec'.format(e - s))

        # output bbox identities
        s = time.time()
        outputs = []
        for track in self.tracker.tracks:
            if not track.is_confirmed() or track.time_since_update > 1:
                continue
            box = track.to_tlwh()
            x1,y1,x2,y2 = self._tlwh_to_xyxy(box)
            track_id = track.track_id
            outputs.append(np.array([x1,y1,x2,y2,track_id], dtype=np.int))
        if len(outputs) > 0:
            outputs = np.stack(outputs,axis=0)
        e = time.time()
        # print('post time: {} sec'.format(e - s))
        return outputs


    """
    TODO:
        Convert bbox from xc_yc_w_h to xtl_ytl_w_h
    Thanks JieChen91@github.com for reporting this bug!
    """
    @staticmethod
    def _xywh_to_tlwh(bbox_xywh):
        if isinstance(bbox_xywh, np.ndarray):
            bbox_tlwh = bbox_xywh.copy()
        elif isinstance(bbox_xywh, torch.Tensor):
            bbox_tlwh = bbox_xywh.clone()
        bbox_tlwh[:,0] = bbox_xywh[:,0] - bbox_xywh[:,2]/2.
        bbox_tlwh[:,1] = bbox_xywh[:,1] - bbox_xywh[:,3]/2.
        return bbox_tlwh


    def _xywh_to_xyxy(self, bbox_xywh):
        x,y,w,h = bbox_xywh
        x1 = max(int(x-w/2),0)
        x2 = min(int(x+w/2),self.width-1)
        y1 = max(int(y-h/2),0)
        y2 = min(int(y+h/2),self.height-1)
        return x1,y1,x2,y2

    def _tlwh_to_xyxy(self, bbox_tlwh):
        """
        TODO:
            Convert bbox from xtl_ytl_w_h to xc_yc_w_h
        Thanks JieChen91@github.com for reporting this bug!
        """
        x,y,w,h = bbox_tlwh
        x1 = max(int(x),0)
        x2 = min(int(x+w),self.width-1)
        y1 = max(int(y),0)
        y2 = min(int(y+h),self.height-1)
        return x1,y1,x2,y2

    def _xyxy_to_tlwh(self, bbox_xyxy):
        x1,y1,x2,y2 = bbox_xyxy

        t = x1
        l = y1
        w = int(x2-x1)
        h = int(y2-y1)
        return t,l,w,h
    
    def _get_features(self, bbox_xywh, ori_img):
        im_crops = []
        for box in bbox_xywh:
            x1,y1,x2,y2 = self._xywh_to_xyxy(box)
            im = ori_img[y1:y2,x1:x2]
            im_crops.append(im)
        if im_crops:
            features = self.extractor(im_crops)
        else:
            features = np.array([])
        return features
    
    @staticmethod
    def _get_IOU(xyxy_1, xyxy_2):
        inters = max(min(xyxy_1[2], xyxy_2[2]) - max(xyxy_1[0], xyxy_2[0]), 0.0) * \
            max(min(xyxy_1[3], xyxy_2[3]) - max(xyxy_1[1], xyxy_2[1]), 0.0)
        unions = (xyxy_1[2] - xyxy_1[0]) * (xyxy_1[3] - xyxy_1[1]) + \
            (xyxy_2[2] - xyxy_2[0]) * (xyxy_2[3] - xyxy_2[1]) - inters
        return float(inters / unions)


