import rosbag
import scipy 
import numpy as np
import tf

Pandar_transition = [1.15, 0 , 1.96]
Pandar_rotation = [0.54, 1.26, 87.95]
# 转欧拉角为四元数
Pandar_rotation = scipy.spatial.transform.Rotation.from_euler('xyz', Pandar_rotation, degrees=True).as_quat()
Pandar_tf_mat = tf.transformations.translation_matrix(Pandar_transition) @ tf.transformations.quaternion_matrix(Pandar_rotation)
front_extrinsic = np.linalg.inv(np.genfromtxt("/mnt/pool1/V2X_AEB/src/common/Mono3d/configs/FisheyeParam/Rock/front/results_front.csv", delimiter=","))
back_extrinsic = np.linalg.inv(np.genfromtxt("/mnt/pool1/V2X_AEB/src/common/Mono3d/configs/FisheyeParam/Rock/back/results_back.csv", delimiter=","))
right_extrinsic = np.linalg.inv(np.genfromtxt("/mnt/pool1/V2X_AEB/src/common/Mono3d/configs/FisheyeParam/Rock/right/results_right.csv", delimiter=","))
left_extrinsic = np.linalg.inv(np.genfromtxt("/mnt/pool1/V2X_AEB/src/common/Mono3d/configs/FisheyeParam/Rock/left/results_left.csv", delimiter=","))

def load_transform_mat(tf_msg):
    transforms = tf_msg.transforms
    for trans in transforms:
        # import pdb; pdb.set_trace()
        if trans.child_frame_id == "Pandar40":
            old_pandar_transition = trans.transform.translation
            old_pandar_rotation = trans.transform.rotation
            old_pandar_tf_mat = tf.transformations.translation_matrix([old_pandar_transition.x, old_pandar_transition.y, old_pandar_transition.z]) @ \
                tf.transformations.quaternion_matrix([old_pandar_rotation.x, old_pandar_rotation.y, old_pandar_rotation.z, old_pandar_rotation.w])
            
            new_pandar_to_front = np.linalg.inv(Pandar_tf_mat) @ old_pandar_tf_mat @ front_extrinsic
            # 保留八位小数
            new_pandar_to_front = np.linalg.inv(new_pandar_to_front)
            new_pandar_to_front = np.round(new_pandar_to_front, 8)

            np.savetxt("results_front.csv", new_pandar_to_front , delimiter=",")

            new_pandar_to_back = np.linalg.inv(Pandar_tf_mat) @ old_pandar_tf_mat @ back_extrinsic
            new_pandar_to_back = np.linalg.inv(new_pandar_to_back)
            new_pandar_to_back = np.round(new_pandar_to_back, 8)
            np.savetxt("results_back.csv", new_pandar_to_back , delimiter=",")

            new_pandar_to_right = np.linalg.inv(Pandar_tf_mat) @ old_pandar_tf_mat @ right_extrinsic
            new_pandar_to_right = np.linalg.inv(new_pandar_to_right)
            new_pandar_to_right = np.round(new_pandar_to_right, 8)
            np.savetxt("results_right.csv", new_pandar_to_right , delimiter=",")

            new_pandar_to_left = np.linalg.inv(Pandar_tf_mat) @ old_pandar_tf_mat @ left_extrinsic
            new_pandar_to_left = np.linalg.inv(new_pandar_to_left)
            new_pandar_to_left = np.round(new_pandar_to_left, 8)
            np.savetxt("results_left.csv", new_pandar_to_left , delimiter=",")

            

if __name__ == '__main__':
    bag = rosbag.Bag("tf_static.bag", 'r')
    for topic, msg, t in bag.read_messages():
        if topic == "/tf_static":
            load_transform_mat(msg)
            break
    bag.close()