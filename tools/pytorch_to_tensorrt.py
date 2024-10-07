#!/usr/bin/env python3
# -*- encoding: utf-8 -*-

import numpy as np
import yaml
import sys
import os
# Add the path to the 'src' directory (parent of common and centerserver)
src_path = '/media/data/V2X_AEB/src'
sys.path.append(src_path)
sys.path.append(os.path.join(src_path, 'common', 'Mono3d'))
from common.Mono3d.models.builder import build_detector

import torch
import onnx_graphsurgeon as gs
from time import time
import onnx
from mmdeploy.utils import Backend
from mmdeploy.apis.utils import to_backend
import logging
from convert import convert_model_to_int32

vehicle = 'hycan'
config_path = '/media/data/V2X_AEB/src/common/Mono3d/configs/models'
config = os.path.join(config_path, 'mv_dfm_{}.yaml'.format(vehicle))
with open(config, 'r') as f:
    config = yaml.safe_load(f)
detector = build_detector(config)
device = "cuda" if torch.cuda.is_available() else "cpu"
# detector.load_state_dict(torch.load(ckpt_path))
detector.to(device)
detector.eval()

height = 480
width = 640

ckpt_path = '/media/data/V2X_AEB/model_ckpt/hycan_mv_fcos3d.pth'
# start to turn torch into onnx
onnx_model_path = os.path.join(ckpt_path.split('.pth')[0] + '.onnx')
dummy_input = torch.randn(1, 4, 3, height, width).to(device)
log_level = logging.INFO
img_metas = [
    dict(
        img_shape=[(height, width, 3)] * 4,
        ori_shape=[(720, 1280, 3)] *4,
        pad_shape=[(height, width, 3)] * 4,
        scale_factor=torch.FloatTensor([0.5, 0.5]).to(device),
        flip=False,
        keep_ratio=True,
        num_views = 4,
        num_ref_frames = 0,
        direction = ['front', 'back', 'left', 'right'],
        pad_size_divisor = 16,
    )
]

# with torch.no_grad():
#     torch.onnx.export(detector, (dummy_input, img_metas, False), 
#                         onnx_model_path, verbose=True, opset_version=12,
#                         input_names=['input', 'img_metas', 'return_loss'],
#                         output_names=['boxes_3d'],
#                         do_constant_folding=False)

# graph = gs.import_onnx(onnx.load(onnx_model_path))

# Fold constants in the graph using ONNX Runtime. This will replace
# expressions that can be evaluated prior to runtime with constant tensors.
# The `fold_constants()` function will not, however, remove the nodes that
# it replaced - it simply changes the inputs of subsequent nodes.
# To remove these unused nodes, we can follow up `fold_constants()` with `cleanup()`
# graph.fold_constants().cleanup()
# for node in graph.nodes:
#     if node.op == 'TopK' :
#         print(node)
#         k_input = node.inputs[1]
#         if k_input.inputs and isinstance(k_input.inputs[0], gs.ir.node.Node):
#             identity_node = k_input.inputs[0]
#             node.inputs[1] = identity_node.inputs[0]

# # 导出修改后的模型
# onnx.save(gs.export_onnx(graph), onnx_model_path.split('.onnx')[0] + '_fold.onnx')

# print("Onnx model is saved at: ", onnx_model_path.split('.onnx')[0] + '_int32.onnx')

# 简化模型
# onnx_model = onnx.load(onnx_model_path)
# # check model
# valid =  onnx.checker.check_model(onnx_model)
# import pdb; pdb.set_trace()
# print("ONNX model is valid: ", valid)
# simplified_model = onnxsim.simplify(onnx_model)
# simplified_model_path = os.path.join(ckpt_path.split('.pth')[0] + '_simplified.onnx')
# onnx.save(simplified_model, simplified_model_path)
# from onnx import helper
# # 遍历所有的初始权重和节点
# for initializer in onnx_model.graph.initializer:
#     if initializer.data_type == onnx.TensorProto.INT64:
#         initializer.CopyFrom(helper.make_tensor(
#             initializer.name,
#             onnx.TensorProto.INT32,  # 修改为 int32 类型
#             initializer.dims,
#             [int(x) for x in initializer.int64_data]  # 转换数据为 int32
#         ))

# # 遍历所有的输入和输出节点
# for input in onnx_model.graph.input:
#     if input.type.tensor_type.elem_type == onnx.TensorProto.INT64:
#         input.type.tensor_type.elem_type = onnx.TensorProto.INT32

# for output in onnx_model.graph.output:
#     if output.type.tensor_type.elem_type == onnx.TensorProto.INT64:
#         output.type.tensor_type.elem_type = onnx.TensorProto.INT32

# # 保存修改后的模型
# simplified_model_path = onnx_model_path.split('.onnx')[0] + '_int32.onnx'

# backend = Backend.TENSORRT
# backend_config = {
#     'backend_config': {
#         'common_config': {
#             'max_workspace_size': 1 << 36,
#         },
#         'fp16_mode' : True,
#         'model_inputs' : [{
#             # "save_file": onnx_model_path.split('.onnx')[0] + '.engine',
#             "input_shapes": {
#                 "input": {
#                     "min_shape": [1, 4, 3, height, width],
#                     "opt_shape": [1, 4, 3, height, width],
#                     "max_shape": [1, 4, 3, height, width],
#                 }
#             }
#         }]
#     }
# }

# trt_model_file = to_backend(
#     backend,
#     [onnx_model_path.split('.onnx')[0] + '_folded.onnx'],
#     work_dir=onnx_model_path.split('hycan_mv_fcos3d')[0],
#     deploy_cfg=backend_config,
#     log_level=log_level,
#     device=device)

# print("TensorRT model is saved at: ", trt_model_file)
