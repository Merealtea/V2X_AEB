import rosbag  
import rospy
import os 
import sys
import cv2
import numpy as np
from hycan_msgs.msg import DetectionResults, Box3D

node = rospy.init_node('bbox_to_msg', anonymous=True)

bbox_file_dir = "/mnt/pool1/catkin_livox_multi/bbox2024-06-25-21-47-20"
box_files = os.listdir(bbox_file_dir)
box_stamp = np.array(sorted([float(box_file.split('_')[-1].split('.txt')[0]) for box_file in box_files]))
time_diff = None
bag_file = "/mnt/pool1/V2X_AEB/data/2024-09-10_10-21-21.bag"
new_bag_file = bag_file.split(".bag")[0] + "_new_box.bag" 
bag = rosbag.Bag(bag_file, 'r')
new_bag = rosbag.Bag(new_bag_file, 'w')

hycan_topic = '/hycan/detection_results'
rock_topic = '/rock/detection_results'

length = 4.6
width = 1.9
bounding_box = np.array([[length/2, width/2],
                        [length/2, -width/2],
                        [-length/2, -width/2],
                        [-length/2, width/2],
                        [length/2, width/2],])

bounding_boxes = []
yaws = []
points = []
i = 0
max_frame = 1600
min_x , min_y = 1e9, 1e9
max_x, max_y = -1e9, -1e9
depth_range = 10

def turn_to_new_msg(msg, vehicle_id):
    new_msg = DetectionResults()
    new_msg.localization.utm_x = msg.localization.utm_x
    new_msg.localization.utm_y = msg.localization.utm_y
    new_msg.localization.heading = msg.localization.heading
    new_msg.sender.stamp = msg.sender.stamp
    new_msg.reciever.stamp = msg.reciever.stamp
    new_msg.image_stamp = msg.image_stamp
    new_msg.num_boxes = msg.num_boxes
    new_msg.vehicle_id = vehicle_id
    return new_msg

def callback(msg):
    global min_x, min_y, max_x, max_y, time_diff, box_stamp, i
    new_msg = turn_to_new_msg(msg, "hycan")
    position = msg.localization.utm_x, \
        msg.localization.utm_y
    heading = msg.localization.heading
    if min_x > position[0]:
        min_x = position[0]
    if min_y > position[1]:
        min_y = position[1]
    if max_x < position[0]:
        max_x = position[0]
    if max_y < position[1]:
        max_y = position[1]

    image_stamp = msg.image_stamp.to_sec()
    if time_diff is None:
        time_diff = image_stamp - box_stamp[0]
        box_stamp += time_diff

    min_diff, min_idx = np.min(np.abs(box_stamp - image_stamp)), np.argmin(np.abs(box_stamp - image_stamp))
    bboxes = []
    if min_diff > 0.03:
         bboxes = []
    else:
        with open(os.path.join(bbox_file_dir, "{:.6f}.txt".format(box_stamp[min_idx] - time_diff)), 'rb') as f:
            for line in f:
                # 去除末尾的换行符并输出
                line = line.strip().decode('utf-8')
                bbox = [float(str(num)) for num in line.split(" ")]
                if bbox[3] < 0.15 or bbox[4] < 0.15 or bbox[5] < 0.5:
                    continue
                bboxes.append(bbox)

            if len(bboxes) != 0:
                bboxes = np.array(bboxes)
                depth = np.sqrt(bboxes[:,0]**2 + bboxes[:,1]**2)
                bboxes = bboxes[depth < depth_range]
                depth = depth[depth < depth_range]

                bboxes[:,:2] = bboxes[:,:2] + np.array(position).reshape(1, 2)
            else:
                bboxes = np.zeros((0, 9))

    
    # import pdb; pdb.set_trace()
    # bounding_box = np.dot(bounding_box, np.array([[np.cos(heading), -np.sin(heading)],
    #                         [np.sin(heading), np.cos(heading)]]).T) 
    # bounding_box += np.array(position).reshape(1, 2)
    # yaws.append(position[2])
    # points.append(position)
    # bounding_boxes.append(bounding_box)

    for box in bboxes:
        box_msg = Box3D()
        box_msg.center_x = box[0]
        box_msg.center_y = box[1]
        box_msg.center_z = box[2]
        box_msg.width = box[3]
        box_msg.length = box[4]
        box_msg.height = box[5]
        box_msg.heading = box[6]
        new_msg.box3d_array.append(box_msg)
    new_msg.num_boxes = len(bboxes) 
    new_bag.write("detection_results", new_msg, msg.reciever.stamp)

# import pdb; pdb.set_trace()
for topic, msg, t in bag.read_messages(topics=[hycan_topic, rock_topic]):
    if topic == hycan_topic:
        callback(msg)
    if topic == rock_topic:
        new_msg = turn_to_new_msg(msg, "rock")
        new_bag.write("detection_results", new_msg, msg.reciever.stamp)
bag.close()
new_bag.close()