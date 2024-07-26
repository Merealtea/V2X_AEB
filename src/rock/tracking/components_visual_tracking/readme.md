#
## 车牌识别
使用[HyperLPR开源库](https://github.com/szad670401/HyperLPR)
- 注意该Python第三方库当前版本在使用时会报`cv2 has no module named estimateRigidTransform`的错误，原因时该函数已被Opencv-python废弃。按照[issue#334](https://github.com/szad670401/HyperLPR/issues/344)修改即可