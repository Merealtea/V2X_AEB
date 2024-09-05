import numpy as np
from ...src.common.Mono3d.configs.FisheyeParam.cam_model import CamModel
import cv2

vehicle_center = np.array([16.3269, 14.4592, 0]) # hycan
center_to_rear_center = np.array([1.426, 0.15, 0.0])
vehicle_rot = np.array([0, 0, -0.999185, 0.040353])

lidar_calib = np.array([[1.0, 0.0, 0.0, 0.0],
                        [0.0, 1.0, 0.0, 0.0],
                        [0.0, 0.0, 1.0, 0.0],
                        [0.0, 0.0, 0.0, 1.0]])

# turn lidar carlib into a vehicle rear center coordinate system
lidar_to_rear = np.linalg.inv(lidar_calib)
lidar_points = np.random.rand(100, 3).T
lidar_points = np.vstack((lidar_points, np.ones(100)))
lidar_points = np.dot(lidar_to_rear, lidar_points)

vehicle = "Hycan"

# lidar points to camera coordinate system
cam_models = {"left": CamModel("left", vehicle), 
            "right": CamModel("right", vehicle),
            "front": CamModel("front", vehicle), 
            "back": CamModel("back", vehicle)}

for key in cam_models:
    cam_points = cam_models[key].world2cam(lidar_points)
    image_points = cam_models[key].cam2image(cam_points)

    # draw points to image
    image = cv2.imread("{}.jpg".format(key))
    for point in image_points.T:
        cv2.circle(image, (int(point[0]), int(point[1])), 2, (0, 0, 255), -1)
    cv2.imwrite("{}.jpg".format(key), image)



