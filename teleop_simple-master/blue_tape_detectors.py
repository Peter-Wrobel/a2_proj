import cv2
import time
import numpy as np
import sys

class Tape_Detector:
    def __init__(self, blue_thresh=10):
        self.thresh = blue_thresh

    def search_stopline(self, image):
        """
        returns: bool if found, center of stopline
        """
        image = image[:,:,0]
        np.less(image, self.thresh, out=image, dtype=np.uint8)
        
        # interested search region must be tuned
        # range to search in -> activation = np.sum(image[260:400,180:460])
        # activation threshold needs tuning as well
        
        # performing convolution with a kernel that is diamond-shaped
        activations = np.zeros((4,5))
        for i, col in enumerate(range(340, 490, 30)):
            # min = 0
            # max = 479
            for j, row in enumerate(range(180, 260, 20)):
                if image[row+10, col+10]+image[row+10, col+15]+image[row+10, col+20] >= 2:
                    # 2 of 3 points were blue
                    activations[j, i] = 1
        
        # if np.sum(activations) > 4:
        for row in activations.shape[0]:
            consecutive = 0
            for col in activations.shape[1]:
                if activations[row, col]:
                    # 2 of 3 points were blue
                    consecutive += 1
                else:
                    consecutive = 0
            if consecutive > 2:
                return True, ((row)*20 + 180, (col-1)*30 + 340)
            

        return False, None

    def search_post(self, image):
        """
        returns: bool if found, center of post
        """
        image = image[:,:,0]
        np.less(image, 10, out=image, dtype=np.uint8)
        
        # interested search region must be tuned
        activation = np.sum(image[80:400,480:640])
        # activation threshold needs tuning as well
        if activation > 100:
            # found post
        pass