import sys
from numpy.lib.arraypad import pad
sys.path.insert(0, './YOLOX')
import torch
import numpy as np
import cv2
import tensorrt
from torch2trt import torch2trt, TRTModule

import time

from YOLOX.yolox.data.data_augment import preproc
from YOLOX.yolox.data.datasets import COCO_CLASSES
from YOLOX.yolox.exp.build import get_exp_by_name
from YOLOX.yolox.utils import postprocess
from utils.visualize import vis

import pycuda.gpuarray as gpuarray
import pycuda.autoinit
import pycuda.driver as cuda


COCO_MEAN = (0.485, 0.456, 0.406)
COCO_STD = (0.229, 0.224, 0.225)

def preproc(image, input_size, mean, std, swap=(2, 0, 1)):
    # s = time.time()
    if len(image.shape) == 3:
        padded_img = np.ones((input_size[0], input_size[1], 3)) * 114.0
    else:
        padded_img = np.ones(input_size) * 114.0
    img = np.array(image)
    r = min(input_size[0] / img.shape[0], input_size[1] / img.shape[1])
    resized_img = cv2.resize(
        img,
        (int(img.shape[1] * r), int(img.shape[0] * r)),
        interpolation=cv2.INTER_LINEAR,
    ).astype(np.float32)
    padded_img[: int(img.shape[0] * r), : int(img.shape[1] * r)] = resized_img
    # e = time.time()
    # print('detec pre padd time: {} sec'.format(e - s))

    # s = time.time()
    padded_img = padded_img[:, :, ::-1]
    padded_img /= 255.0
    if mean is not None:
        padded_img -= mean
    if std is not None:
        padded_img /= std
    # e = time.time()
    # print('detec pre norm time: {} sec'.format(e - s))
    # s = time.time()
    padded_img = padded_img.transpose(swap)
    padded_img = np.ascontiguousarray(padded_img, dtype=np.float32)
    # e = time.time()
    # print('detec pre post time: {} sec'.format(e - s))
    return padded_img, r




class Detector():
    """ 图片检测器 """
    def __init__(self, model='yolox-s', ckpt='yolox_s.pth.tar', trt=False):
        super(Detector, self).__init__()



        self.device = torch.device('cuda:0') if torch.cuda.is_available() else torch.device('cpu')
        self.trt = trt

        self.exp = get_exp_by_name(model)
        self.test_size = self.exp.test_size  # TODO: 改成图片自适应大小
        self.model = self.exp.get_model()
        self.model.to(self.device)
        self.model.eval()
        checkpoint = torch.load(ckpt, map_location="cpu")
        self.model.load_state_dict(checkpoint["model"])
        self.decoder = None
        if self.trt:
            self.model.head.decode_in_reference = False
            self.decoder = self.model.head.decode_outputs
            model_trt = TRTModule()
            model_trt.load_state_dict(torch.load('./weights/yolox_s_trt.pth'))
            x = torch.ones(1, 3, self.exp.test_size[0], self.exp.test_size[1]).cuda()
            self.model(x)
            self.model = model_trt
        



    def detect(self, raw_img, visual=True, conf=0.5):
        s = time.time()
        info = {}
        img, ratio = preproc(raw_img, self.test_size, COCO_MEAN, COCO_STD)
        info['raw_img'] = raw_img
        info['img'] = img

        img = torch.from_numpy(img).unsqueeze(0)
        img = img.to(self.device)
        e = time.time()
        # print('detect pre time: {} sec'.format(e - s))

        s = time.time() 
        with torch.no_grad():
            outputs = self.model(img)
            if self.decoder is not None:
                outputs = self.decoder(outputs, dtype=outputs.type())
            outputs = postprocess(
                outputs, self.exp.num_classes, self.exp.test_conf, self.exp.nmsthre  # TODO:用户可更改
            )
            outputs = outputs[0].cpu().numpy()
        e = time.time()
        # print('detect forward time: {} sec'.format(e - s))

        s = time.time()
        info['boxes'] = outputs[:, 0:4]/ratio
        info['scores'] = outputs[:, 4] * outputs[:, 5]
        info['class_ids'] = outputs[:, 6]
        info['box_nums'] = outputs.shape[0]
        # 可视化绘图
        if visual:
            info['visual'] = vis(info['raw_img'], info['boxes'], info['scores'], info['class_ids'], conf, COCO_CLASSES)
        e = time.time()
        # print('detect post time: {} sec'.format(e - s))
        return info






if __name__=='__main__':
    detector = Detector()
    img = cv2.imread('dog.jpg')
    img_,out = detector.detect(img)
    print(out)
