import time
import cv2
import numpy as np
import math
from collections import deque

def euclidian_distance(point_a, point_b):
  """
  point_a: (r, c) tuple
  point_b: (r, c) tuple
  """
  return math.sqrt((point_a[0] - point_b[0])**2 + (point_a[1] - point_b[1])**2)

def history_point_to_cv(point, flip_xy=False):
  """
  flip_xy: whether to flip the x and y coordinates when drawing on the image
  """
  tuple_as_int = (int(point[0]), int(point[1]))
  return tuple_as_int[::-1] if flip_xy else tuple_as_int

class Tracker:
  def __init__(self, position):
    """
    position: tuple of form (r, c)
    """
    self.position = position
    self.history = deque([position]) # [most recent => longest ago]
    self.last_update_time = time.time()
    random_color = (list(np.random.choice(range(256), size=3)))  
    self.color =[int(random_color[0]), int(random_color[1]), int(random_color[2])]

  def __str__(self):
    return "Cross Centered at: {}, {}\n".format(*self.position)

  def __repr__(self):
    return "Cross Centered at: {}, {}\n".format(*self.position)

  def update_position(self, position):
    """
    position: tuple of form (r, c)
    """
    self.position = position
    self.history.appendleft(position)

    # Remove oldest item from history if deque is too full
    if len(self.history) > 20:
      self.history.pop()

    self.last_update_time = time.time()

  def distance_to_point(self, point):
    """
    Returns the Euclidian distance between the current center
    of the cross and the point passed in
    point: tuple of form (r, c)
    """
    return euclidian_distance(self.position, point)

  def tracker_is_alive(self):
    """
    Returns whether a tracker is still alive or not
    """
    return time.time() - self.last_update_time < 1

  def draw_center(self,img):
    img[self.position[0]-2:self.position[0]+2,self.position[1]-2:self.position[1]+2] = [0,0,255]

  def draw_trail_on_image(self, image, flip_xy=False):
    """
    flip_xy: whether to flip the x and y coordinates when drawing on the image
    """

    cv2.circle(image, history_point_to_cv(
        self.history[0], flip_xy=flip_xy), 5, self.color, -1)
    
    # compute the thickness of the line and
    thickness=np.linspace(5, 1, len(self.history), dtype=np.uint8)

    # loop over the set of tracked points
    for i in range(1, len(self.history)):
      # if either of the tracked points are None, ignore
      # them
      if self.history[i - 1] is None or self.history[i] is None:
        continue

      # draw the connecting lines
      cv2.line(image, history_point_to_cv(
          self.history[i - 1], flip_xy=flip_xy), history_point_to_cv(self.history[i], flip_xy=flip_xy), self.color, thickness[i])

