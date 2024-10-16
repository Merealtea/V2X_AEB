from multiprocessing import shared_memory, resource_tracker
import struct
import numpy as np

class Images_Shared_Memory():
    def __init__(self, shm_name, camera_num):
        self.shm = shared_memory.SharedMemory(shm_name)
        self.camera_num = camera_num
        
        resource_tracker.unregister("/" + shm_name, "shared_memory")
        self.size_each_image = int(self.shm.size / camera_num)
        self.aim = 0
        
    def read_imgs_from_shm(self, select_camera_ids = None):
        """ Read camera frame from the shm
            if camera_id is None, then it will return all the imgs
            else it will return the referred frame image
        """
        imgs = []
        frame_idxs = []
        camera_ids = []
        self.aim = self.aim+1
        
        for i in range(self.camera_num):
            # if i > 1:
            #     break
            j = self.size_each_image * i
            b = struct.unpack("<iiiii", self.shm.buf[j:j+20])

            camera_id = b[0]
            frame_idx = b[1]
            width = b[2]
            height = b[3]
            channel = b[4]
            
            if select_camera_ids is None or camera_id in select_camera_ids:
                array = np.frombuffer(self.shm.buf[j+20:j+20+1920*1080*3], dtype=np.uint8).reshape(1080,1920,3)
                imgs.append(array)
                frame_idxs.append(frame_idx)
                camera_ids.append(camera_id)
            
        return frame_idxs, imgs, camera_ids

    