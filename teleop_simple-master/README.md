Imporant - Readme.

###########Image detection #############

###IMPORTANT### - 'h' and 'v' keys commented out. To flip image, properly, look at line 65 - adjust to camera settings
###IMPORTANT###  LOOK FOR "CHANGE ME" TO SEE WHERE CHANGEABLE PARAMETERS ARE IN FILES "blue_tape_detectors.py" and "mbot_teleop_simple.py"


1. The cross
   a.  should always show up in the proper coordinates, regardless of camera orientation. If not, let me know
   b.  My training image is "cross_ugly.png". Default is "cross_train.png". If cross detection is not working well,
       add your own photo to teleop_simple-master folder, and change line. 

       To add your own photo to repo from your laptop, run 
            scp {YOUR_CROSS.PNG} pi@192.168.3.1:/home/pi/A2/a2_proj/teleop_simple-master/ 


2. The Tape. 
   a. use "self.show_pic(image)" to see how micheal's code sees what it's about to process. 
   b. Look at return values. I have Row length -row_result because of my camera orientation. This return value is used
      to draw the red dot. 

