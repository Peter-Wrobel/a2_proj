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
        img = image[:,:,2].copy()
        np.less(img, self.thresh, out=img, dtype=np.uint8)
        
        # interested search region must be tuned
        # range to search in -> activation = np.sum(image[260:400,180:460])
        # activation threshold needs tuning as well
        
        # performing convolution with a kernel that is diamond-shaped
        activations = np.zeros((8,10))
        for i, col in enumerate(range(190, 490, 30)):
            # min = 0
            # max = 479
            for j, row in enumerate(range(280, 440, 20)):
                print("row:", row,"\tcol:", col)
                if img[row+10,col+10]+img[row+10,col+15]+img[row+10,col+20] >= 2:
                    # 2 of 3 points were blue
                    activations[j, i] = 1
        
        # if np.sum(activations) > 4:
        print("ACTIVATIONS:\n", activations)
        for row in range(activations.shape[0]):
            consecutive = 0
            for col in range(activations.shape[1]):
                if activations[row, col]:
                    # 2 of 3 points were blue
                    print("counted:", row, "\t", col)
                    consecutive += 1
                else:
                    consecutive = 0
                if consecutive > 4:
                    return True, ((col-3)*30 + 190, row*20 + 280)
            

        return False, None

    def search_post(self, image):
        """
        returns: bool if found, center of post
        """
        img = image[:,:,2].copy()
        np.less(img, self.thresh, out=img, dtype=np.uint8)
        
        # interested search region must be tuned
        # range to search in -> activation = np.sum(image[260:400,180:460])
        # activation threshold needs tuning as well
        
        # performing convolution with a kernel that is diamond-shaped
        activations = np.zeros((8,10))
        for i, col in enumerate(range(460, 620, 20)):
            # min = 0
            # max = 479
            for j, row in enumerate(range(220, 400, 30)):
                print("row:", row,"\tcol:", col)
                if img[row+10,col+10]+img[row+15,col+10]+img[row+20,col+10] >= 2:
                    # 2 of 3 points were blue
                    activations[j, i] = 1
        
        # if np.sum(activations) > 4:
        print("ACTIVATIONS:\n", activations)
        for col in range(activations.shape[1]):
            consecutive = 0
            for row in range(activations.shape[0]):
                if activations[row, col]:
                    # 2 of 3 points were blue
                    print("counted:", row, "\t", col)
                    consecutive += 1
                else:
                    consecutive = 0
                if consecutive > 4:
                    return True, ((col+1)*20 + 460, (row-2)*30 + 220)
            

        return False, None
