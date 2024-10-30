import numpy as np
import time
from filterpy.kalman import KalmanFilter
from copy import deepcopy

def linear_assignment(cost_matrix):
  try:
    import lap
    _, x, y = lap.lapjv(cost_matrix, extend_cost=True)
    return np.array([[y[i],i] for i in x if i >= 0]) #
  except ImportError:
    from scipy.optimize import linear_sum_assignment
    x, y = linear_sum_assignment(cost_matrix)
    return np.array(list(zip(x, y)))

def iou_batch_rot(dt_annos_part, gt_annos_part):
    """
    From SORT: Computes IOU between two bboxes in the form [x1,y1,x2,y2]
    """
    loc = gt_annos_part[:,:2]
    dims = gt_annos_part[:, 3:5]
    rots = gt_annos_part[:,6:7] % (2 * np.pi)
    gt_boxes = np.concatenate([loc, dims, rots],
                                axis=1)
    loc = dt_annos_part[:,:2]
    dims = dt_annos_part[:, 3:5]
    rots = dt_annos_part[:, 6:7]
    dt_boxes = np.concatenate([loc, dims, rots],
                                axis=1)

    overlap_part = bev_box_overlap(gt_boxes,
                                    dt_boxes).astype(np.float64)
    print("dt_boxes is ", dt_boxes)
    print("gt_boxes is ", gt_boxes)
    print("overlap is ", overlap_part)
    return overlap_part.T

def iou_batch(bb_test, bb_gt):
  """
  From SORT: Computes IOU between two bboxes in the form [x1,y1,x2,y2]
  """
  bb_gt = np.expand_dims(bb_gt, 0)
  bb_test = np.expand_dims(bb_test, 1)

  xx1 = np.maximum(bb_test[..., 0] - bb_test[..., 3] / 2,
                    bb_gt[..., 0] - bb_gt[..., 3] / 2)
  yy1 = np.maximum(bb_test[..., 1] - bb_test[..., 4] / 2,
                   bb_gt[..., 1] - bb_gt[..., 4] / 2)
  xx2 = np.minimum(bb_test[..., 0] + bb_test[..., 3] / 2, 
                   bb_gt[..., 0] + bb_gt[..., 3] / 2)
  yy2 = np.minimum(bb_test[..., 1 ] + bb_test[..., 4] / 2,
                   bb_gt[..., 1] + bb_gt[..., 4] / 2)
  w = np.maximum(0., xx2 - xx1)
  h = np.maximum(0., yy2 - yy1)
  wh = w * h
  o = wh / (bb_test[..., 3] * bb_test[..., 4] + bb_gt[..., 3] *  bb_gt[..., 4] - wh)     
  return(1-o)  

def distance_matrix(boxes, boxes2):
    """
    Compute the distance matrix between boxes and boxes2
    """
    dist = np.zeros((len(boxes), len(boxes2)))
    for i, box in enumerate(boxes):
        for j, box2 in enumerate(boxes2):
            dist[i, j] = np.linalg.norm(box[:2] - box2[:2])
    return dist

def bev_box_overlap(boxes, qboxes, criterion=-1):
    from rotate_iou import rotate_iou_gpu_eval
    riou = rotate_iou_gpu_eval(boxes, qboxes, criterion)
    return riou

def min_bbox(bbox_width, bbox_height):
    """
    Get the minimum width and height of the bbox
    """
    min_width = 0.4
    min_length = 0.2
    max_width = 0.8
    max_length = 0.4
    if bbox_width < bbox_height:
        bbox_width = min(max(bbox_width, min_length), max_length)
        bbox_height = min(max(bbox_height, min_width), max_width)
    else:
        bbox_width = min(max(bbox_width, min_width), max_width)
        bbox_height = min(max(bbox_height, min_length), max_length)
    return bbox_width, bbox_height

def convert_bbox_to_bev(bbox):
  """
    The 3d bbox is in the form [x,y,z,l,w,h,theta] 
    The 2d bbox is in the form [x,y,w,h,theta]
  """
  x, y, l, w, theta = bbox[0], bbox[1], bbox[3], bbox[4], bbox[6]
  w, l = min_bbox(w, l)
  return np.array([x, y, l, w, theta]).reshape((5, 1))

class KalmanBoxTracker(object):
  """
  This class represents the internal state of individual tracked objects observed as bbox.
  """
  count = 0
  def __init__(self,bbox, timestamp):
    """
    Initialises a tracker using initial bounding box.
    """
    #define constant velocity model
    # x = [x,y, l, w, theta, dx, dy, dtheta]
    # z = [x,y,l,w, theta]
    self.kf = KalmanFilter(dim_x=10, dim_z=5) 
    self.kf.R[2:,2:] *= 10.
    self.kf.P[5:,5:] *= 1000. #give high uncertainty to the unobservable initial velocities
    self.kf.P *= 10.
    # self.kf.Q[-1,-1] *= 0.01
    self.kf.Q[5:,5:] *= 0.01
    self.center_z = bbox[2]
    self.height = bbox[5] 

    self.kf.x[:5] = convert_bbox_to_bev(bbox)

    init_velocities = 0
    heading = bbox[6]
    x_vel = init_velocities * np.cos(heading + np.pi)
    y_vel = init_velocities * np.sin(heading + np.pi)
    self.kf.x[5] = x_vel
    self.kf.x[6] = y_vel
    self.time_since_update = 0
    self.track_timestamp = timestamp # The real time stamp of the track
    self.update_timestamp = time.time() # The time stamp of the last update
    self.id = KalmanBoxTracker.count
    KalmanBoxTracker.count += 1
    self.history = []
    self.age = 0
    self.hit_streak = 0

  def update(self,bbox, timestamp):
    """
    Updates the state vector with observed bbox.
    """
    self.hit_streak += 1
    td = timestamp - self.track_timestamp
    self.time_since_update = 0

    H = np.array([[1,0,0,0,0,td,0,0,0,0],
                  [0,1,0,0,0,0,td,0,0,0],    
                  [0,0,1,0,0,0,0,td,0,0],
                  [0,0,0,1,0,0,0,0,td,0],
                  [0,0,0,0,1,0,0,0,0,td]])
    self.kf.update(convert_bbox_to_bev(bbox), H=H)
    self.track_timestamp = timestamp
    self.update_timestamp = time.time()

  def predict(self, timestamp):
    """
    Advances the state vector and returns the predicted bounding box estimate.
    """
    td = timestamp - self.track_timestamp
    F = np.array([[1,0,0,0,0,td,0,0,0,0],
                  [0,1,0,0,0,0,td,0,0,0],
                  [0,0,1,0,0,0,0,td,0,0],
                  [0,0,0,1,0,0,0,0,td,0],  
                  [0,0,0,0,1,0,0,0,0,td],
                  [0,0,0,0,0,1,0,0,0,0],
                  [0,0,0,0,0,0,1,0,0,0],
                  [0,0,0,0,0,0,0,1,0,0],
                  [0,0,0,0,0,0,0,0,1,0],
                  [0,0,0,0,0,0,0,0,0,1]])
    self.kf.predict(F= F)
    if(self.time_since_update>0):
      self.hit_streak = 0
    self.time_since_update += td
    self.track_timestamp = timestamp
    self.update_timestamp = time.time()
    self.kf.x[3], self.kf.x[2] = min_bbox(self.kf.x[3], self.kf.x[2])
    return self.kf.x[:5]

  def get_state(self):
    """
    Returns the current bounding box estimate.
    """
    x = self.kf.x[:5].reshape((5, 1))
    new_x = np.zeros((7, 1))
    new_x[:2] = x[:2]
    new_x[2] = self.center_z
    new_x[4], new_x[3] = min_bbox(x[3], x[2])
    new_x[5] = self.height
    new_x[6] = x[4]
    return new_x


  def predict_without_update(self, timestamp):
    """
    Advances the state vector without updating.
    """
    td = timestamp - self.track_timestamp
    H = np.array([[1,0,0,0,0,td,0,0,0,0],
                  [0,1,0,0,0,0,td,0,0,0],    
                  [0,0,1,0,0,0,0,td,0,0],
                  [0,0,0,1,0,0,0,0,td,0],
                  [0,0,0,0,1,0,0,0,0,td]])
    predict_x = np.dot(H, self.kf.x) 
    new_predcit_x = np.zeros((7, 1))
    new_predcit_x[:2] = predict_x[:2]
    new_predcit_x[2] = self.center_z
    new_predcit_x[4], new_predcit_x[3] = min_bbox(predict_x[3], predict_x[2])
    new_predcit_x[5] = self.height
    new_predcit_x[6] = predict_x[4]
    print("new predict x is ", new_predcit_x)
    return new_predcit_x

def associate_detections_to_trackers(detections,trackers,cost_threshold = 0.3):
  """
  Assigns detections to tracked object (both represented as bounding boxes)

  Returns 3 lists of matches, unmatched_detections and unmatched_trackers
  """
  if(len(trackers)==0):
    return np.empty((0,2),dtype=int), np.arange(len(detections)), np.empty((0,5),dtype=int)

  cost_matrix = distance_matrix(detections, trackers)
  print("Cost matrix is ", cost_matrix)

  if min(cost_matrix.shape) > 0:
    a = (cost_matrix < cost_threshold).astype(np.int32)
    if a.sum(1).max() == 1 and a.sum(0).max() == 1:
        matched_indices = np.stack(np.where(a), axis=1)
    else:
      matched_indices = linear_assignment(cost_matrix)
  else:
    matched_indices = np.empty(shape=(0,2))

  unmatched_detections = []
  for d, det in enumerate(detections):
    if(d not in matched_indices[:,0]):
      unmatched_detections.append(d)
  unmatched_trackers = []
  for t, trk in enumerate(trackers):
    if(t not in matched_indices[:,1]):
      unmatched_trackers.append(t)

  #filter out matched with low IOU
  matches = []
  for m in matched_indices:
    if(cost_matrix[m[0], m[1]] > cost_threshold):
      unmatched_detections.append(m[0])
      unmatched_trackers.append(m[1])
    else:
      matches.append(m.reshape(1,2))
  if(len(matches)==0):
    matches = np.empty((0,2),dtype=int)
  else:
    matches = np.concatenate(matches,axis=0)

  return matches, np.array(unmatched_detections), np.array(unmatched_trackers)


class Sort(object):
  def __init__(self, vehicle_id, localization, max_age=1, min_hits=3, iou_threshold=0.7):
    """
     Every vehicle has its own SORT tracker
    """
    self.vehicle_id = vehicle_id
    self.max_age = max_age
    self.frame_count = 0
    self.min_hits = min_hits
    self.iou_threshold = iou_threshold
    self.trackers = []
    self.start_timestmap = time.time()
    self.current_timestamp = self.start_timestmap
    self.localiztion = localization

  def update(self, localization, dets, det_timestamp):
    """
    Params:
      dets - a numpy array of detections in the format [[x1,y1,x2,y2,score],[x1,y1,x2,y2,score],...]
    Requires: this method must be called once for each frame even with empty detections (use np.empty((0, 5)) for frames without detections).
    Returns the a similar array, where the last column is the object ID.

    NOTE: The number of objects returned may differ from the number of detections provided.
    """
    self.localiztion = localization
    self.current_timestamp = time.time()
    # get predicted locations from existing trackers.
    trks = np.zeros((len(self.trackers), 7))
    to_del = []
    updated_res = {}
    track_res = {}
    res_dets = deepcopy(dets)
    self.frame_count += 1
    for t, trk in enumerate(trks):
      pos = self.trackers[t].predict(det_timestamp)
      # prediction is in the form [x,y,s,r, yaw] in bev view
      trk[:] = [pos[0], pos[1], 0, pos[2], pos[3], 0, pos[4]]
      if np.any(np.isnan(pos)):
        to_del.append(t)
    trks = np.ma.compress_rows(np.ma.masked_invalid(trks))
    for t in reversed(to_del):
      self.trackers.pop(t)
    matched, unmatched_dets, unmatched_trks = associate_detections_to_trackers(dets, trks, self.iou_threshold)

    # update matched trackers with assigned detections
    for m in matched:
      self.trackers[m[1]].update(dets[m[0], :], det_timestamp)
      updated_res[self.trackers[m[1]].id] = res_dets[m[0], :]

    # create and initialise new trackers for unmatched detections
    for i in unmatched_dets:
        trk = KalmanBoxTracker(dets[i,:], det_timestamp)
        self.trackers.append(trk)
        updated_res[trk.id] = res_dets[i, :]
        print("New tracker is created")
    i = len(self.trackers)
    for trk in reversed(self.trackers):
        print("hit times is ", trk.hit_streak)
        i -= 1
        # remove dead tracklet
        if(trk.time_since_update > self.max_age):
          self.trackers.pop(i)
        elif trk.hit_streak >= self.min_hits:
          track_res[trk.id] = trk.get_state().reshape((7, 1))
    return updated_res, track_res

  def predict(self, timestamp):
    """
    Predict the state of the trackers
    """
    predcitions = {}
    for tracker in self.trackers:
        if timestamp - tracker.update_timestamp < self.max_age:
          position = tracker.predict_without_update(timestamp)
          predcitions[tracker.id] = position
    return predcitions