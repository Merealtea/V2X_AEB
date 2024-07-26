# 评测指标
---
## Visual tracking
- precision_iou
- precision_centerdis
- AOR(average overlap rate)： 对所有iou取均值
- APE(average pixel error)： 中心像素距离取均值
- success plot：选取不同iou阈值下precision连接成的曲线
---
## 3D tracking
- ALE(average location error)：box中心点距离取均值
- Precision plot：precision 随着ALE阈值的变化曲线
- yaw error