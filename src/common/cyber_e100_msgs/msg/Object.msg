std_msgs/Header header

## Status
uint32 sensor_type    # 0:Lidar, 1:Camera, 2:Radar, 3:Scala, 4:Fusion
uint32 object_id      # Unique identifier for each object(only for track mode)
uint32 object_type    # 0:Unknown, 1:Pedestrian, 2:Cone, 3:Cyclist, 4:Car, 5:Truck, 6:Bus
float32 object_score  # Indicating confidence in detection
uint32 track_status   # 0:Untracked, 1:New, 2:Tracking, 3:Dead
uint32 track_age      # Continuous tracking time
uint32 motion_status  # 0:Unknown, 1:Static, 2:Moving

## 3D Bounding Box
geometry_msgs/Pose pose             # Position and orientation
geometry_msgs/Vector3 dimensions    # Size
geometry_msgs/Twist velocity        # Linear and angular velocity
geometry_msgs/Twist acceleration    # Linear and angular acceleration
geometry_msgs/Polygon polygon       # Polygon
sensor_msgs/PointCloud2 point_cloud # Point Cloud
