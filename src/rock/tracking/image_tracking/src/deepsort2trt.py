import torch
import tensorrt as trt
from torch2trt import torch2trt

import os
# os.chdir(os.path.abspath(os.path.dirname(__file__)))
from deep_sort.deep_sort.deep.model import Net

model = Net(reid=False)
ckpt_path = os.path.join(os.path.abspath(os.path.dirname(__file__)), 'deep_sort/deep_sort/deep/checkpoint/ckpt.t7')
ckpt = torch.load(ckpt_path, map_location='cpu')

model.load_state_dict(ckpt['net_dict'])
model.eval()
model.cuda()

x = torch.ones(2, 3, 128, 64).cuda()
model_trt = torch2trt(
    model,
    [x],
    fp16_mode=True,
    log_level=trt.Logger.INFO,
    max_workspace_size=(1<<32),
    max_batch_size=256
)

torch.save(model_trt.state_dict(), os.path.join(os.path.abspath(os.path.dirname(__file__)), 'weights/deepsort_trt.pth'))