# python orb_detector.py -i ./images/iphone_cross.png
import numpy as np
import math
import cv2
import argparse
import time
import matplotlib.pyplot as plt
from tracker import Tracker
from tracker import history_point_to_cv

def euclidian_distance(point_a, point_b):
  """
  point_a: (r, c) tuple
  point_b: (r, c) tuple
  """
  sum = 0
  for i in range(len(point_a)):
    sum += (point_a[i] - point_b[i])**2
  
  return math.sqrt(sum)




class ORBTracker(Tracker):
  def __init__(self, keypoint, descriptor):
    super().__init__(keypoint.pt)
    self.keypoint = keypoint
    self.descriptor = descriptor

  def distance_to_descriptor(self, descriptor):
    """
    Calculates the distance between the descriptor for the tracker
    and the passed in descriptor. Uses NORM_HAMMING, which is recommended
    when using ORB descriptors.
    """
    return cv2.norm(self.descriptor, descriptor, cv2.NORM_HAMMING)

  def distance_to_point(self, keypoint):
    """
    Overridden distance to point method that takes in a keypoint
    instead of a (r, c) tuple
    """
    return super().distance_to_point(keypoint.pt)

  def update_position(self, keypoint, descriptor):
    """
    Updates the position with a new keypoint and descriptor
    """
    self.keypoint = keypoint
    self.descriptor = descriptor
    super().update_position(keypoint.pt)

class Cross_ORB:
    def __init__(self, kp, descriptors):

        # Coordinate
        self.kp = kp

        # Feature Vector
        self.descriptors = descriptors




"""
This class just searches for ORB features and draws them on the image
"""
class ORBDetector:
    def __init__(self, debug=False):
        # Initiate ORB detector
        self.orb = cv2.ORB_create(nfeatures=40)

        # A list of ORBTrackers
        self.trackers = []

        # Cross reference
        self.cross_orb = None

    def find_keypoints_descriptors(self, image):
        """
        Find the keypoints and descriptors with ORB
        @returns (keypoints, descriptors)
        """
        grey_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        return self.orb.detectAndCompute(grey_image, None)

    def get_keypoint_descriptor_nparrays(self):
      """
      Gets all the descriptors and keypoints from the trackers
      @returns: a tuple of form (np array of keypoints, np array of descriptos)
      """
      descriptors = [tracker.descriptor for tracker in self.trackers]
      keypoints = [tracker.keypoint for tracker in self.trackers]

      return np.array(keypoints), np.array(descriptors)

    def merge_points(self, raw_points):
      """
      Uses Euclidian distance to ensure there aren't
      cross centers too close to each other
      centers: list of (r, c) that represent the proposed
              centers of the crosses
      """
      # Macros for indexes
      ROW = 0
      COL = 1
      WEIGHT = 2
      DIST_THRESH = 30

      centers = []
      for r in raw_points:
        centers.append((r[ROW],r[COL], 1))

      outer = 0
      while outer < len(centers)-1:
        inner = outer + 1
        change = False
        while inner < len(centers):
          if euclidian_distance(centers[inner], centers[outer]) < DIST_THRESH:
            r_new = (centers[inner][ROW] + centers[outer][ROW]) // 2
            c_new = (centers[inner][COL] + centers[outer][COL]) // 2
            w_new = centers[inner][WEIGHT]+centers[outer][WEIGHT]
            centers.pop(outer)
            centers.pop(inner -1)
            centers.append((r_new, c_new,w_new))
            inner -=1
            change = True
          inner+=1
        if change:
          outer = 0
        else:
          outer +=1
          
      return centers


    def read_cross(self, cross_img):
        kp, descriptors = self.find_keypoints_descriptors(cross_img)
        print("cross desc = ", len(descriptors))
        self.cross_orb =  Cross_ORB(kp, descriptors)


    def show_orb_features(self, image):      
        # Get keypoints and descriptors for image

        HOR_LOW = 90
        HOR_HIGH = image.shape[0] - HOR_LOW
        VERT_LOW = 100
        VERT_HIGH = image.shape[1] - VERT_LOW

        time_bef = time.time()
        kp, descriptors = self.find_keypoints_descriptors(image[HOR_LOW:HOR_HIGH, VERT_LOW: VERT_HIGH])
        # print ("+++++", time.time()- time_bef, " TIME TOOK: find_keypoints_descriptors+++++")

        # print(image.shape[0], image.shape[1])


        M_DIST_THRESH = 400
        COMB_THRESH = 7

        # for k in kp:
        #   pt = (int(k.pt[0]), int(k.pt[1]))
        #   cv2.circle(image, pt, 5, [255,0,0], -1)
        
        matcher = cv2.BFMatcher() 
        matches = matcher.match(descriptors,self.cross_orb.descriptors)

        # Apply ratio test
        good = []
        for m in matches:
          match_point = (int(kp[m.queryIdx].pt[0]),int(kp[m.queryIdx].pt[1]))
          # cv2.circle(image, match_point, 5, [0,255,0], -1)
          if m.distance<M_DIST_THRESH:
            good.append(m)
            


        orb_centers = []
        orb_orig = []
        for match in good:
          # print(kp[match.queryIdx].pt)
          match_point = (int(kp[match.queryIdx].pt[0]),int(kp[match.queryIdx].pt[1]))
          kpc = self.cross_orb.kp
          joey_point = (int(kpc[match.trainIdx].pt[0]),int(kpc[match.trainIdx].pt[1])) 
          orb_orig.append(joey_point)
          orb_centers.append(match_point)
          # cv2.circle(image, match_point, 5, [255,0,0], -1)

        cross_centers = self.merge_points(orb_centers)

        for x in cross_centers:
          if x[2]>COMB_THRESH:
            cv2.circle(image, (x[0]+VERT_LOW,x[1]+HOR_LOW), 5, [0,0,255], -1)





        return image

def show_pic(Ix):
  plt.figure()
  plt.axis('off')
  plt.imshow(Ix)
  plt.show()


def main(input_path):
  start_time = time.time()
  input_img = cv2.imread(input_path)
  cross_img = cv2.imread("images/cross_train.png")
  detector = ORBDetector(debug=True)
  print("--- %s seconds ---" % (time.time() - start_time))
  detector.read_cross(cross_img)
  detect_with_video()
  print("--- %s seconds ---" % (time.time() - start_time))
  show_pic(output)



def detect_with_video(video_path):
  detector = ORBDetector(debug=True)

  vs = cv2.VideoCapture(video_path)
  
  # allow the camera or video file to warm up
  time.sleep(2.0)

  # keep looping
  frame_count = 0
  while True:
    # grab the current frame
    frame = vs.read()[1]
    
    # if we are viewing a video and we did not grab a frame,
    # then we have reached the end of the video
    if frame is None:
      break

    output = detector.show_orb_features(frame)

    # show the frame to our screen
    cv2.imshow("Frame", output)

    # time.sleep(0.1)

    key = cv2.waitKey(1) & 0xFF
    # if the 'q' key is pressed, stop the loop
    if key == ord("q"):
      break

    frame_count += 1

if __name__ == "__main__":
  # Construct the argument parser and parse the arguments
  ap = argparse.ArgumentParser()
  ap.add_argument("-i", "--input_image",
                  help="Path to the input image")
  ap.add_argument("-v", "--video", help="path to the (optional) video file")
  args = vars(ap.parse_args())
  if not args.get("video", False):
    main(args["input_image"])
  else:
    detect_with_video(args['video'])
