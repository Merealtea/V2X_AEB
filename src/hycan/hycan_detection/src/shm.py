from multiprocessing import shared_memory, resource_tracker
import struct
import numpy as np
import sys

class Images_Shared_Memory():
    def __init__(self, shm_name, camera_num):
        self.shm = shared_memory.SharedMemory(shm_name)
        self.camera_num = camera_num

        byteorder = sys.byteorder
        
        resource_tracker.unregister("/" + shm_name, "shared_memory")
        self.size_each_image = int(self.shm.size / camera_num)
        if byteorder == 'little':
            self.fmt = "=iiiiiiiiifd"
        else:
            self.fmt = "=iiiiiiiiifd"
        self.header_size = struct.calcsize(self.fmt)
   
        
    def read_imgs_from_shm(self, select_camera_ids = None):
        """ Read camera frame from the shm
            if camera_id is None, then it will return all the imgs
            else it will return the referred frame image
        """
        imgs = []
        frame_idxs = []
        camera_ids = []
       
        
        for i in range(self.camera_num):
            # if i > 1:
            #     break
            j = self.size_each_image * i
            b = struct.unpack(self.fmt, self.shm.buf[j:j+self.header_size])

            camera_id = b[0]
            frame_idx = b[1]
            width = b[2]
            height = b[3]
            channel = b[4]
            width_no_pad = b[5]
            height_no_pad = b[6]
            original_width = b[7]
            original_height = b[8]
            ratio = b[9]
            timestamp = b[10]

            if select_camera_ids is None or camera_id in select_camera_ids:
                array = np.frombuffer(self.shm.buf[j+self.header_size:(i+1)*self.size_each_image], 
                                      dtype=np.uint8).reshape(width, height,  channel)
                imgs.append(array.astype(np.float32))
                frame_idxs.append(frame_idx)
                camera_ids.append(camera_id)
        imgs = np.ascontiguousarray(np.stack(imgs))
        return frame_idxs, imgs, camera_ids, width_no_pad, height_no_pad, original_width, original_height, ratio, timestamp

    