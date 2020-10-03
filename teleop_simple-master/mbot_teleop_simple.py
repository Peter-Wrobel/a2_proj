import pygame
from pygame.locals import *
from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2
import time
import numpy as np
import sys
sys.path.append("lcmtypes")
import lcm
from lcmtypes import mbot_motor_pwm_t
FWD_PWM_CMD = 0.3
TURN_PWM_CMD = 0.3
flip_h = 0
flip_v = 0


def main(task_number)
    lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=1")
    pygame.init()
    pygame.display.set_caption("MBot TeleOp")
    screen = pygame.display.set_mode([640,480])
    camera = PiCamera()
    camera.resolution = (640, 480)
    camera.framerate = 32
    rawCapture = PiRGBArray(camera, size=(640, 480))
    time.sleep(0.5)
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        image = frame.array
        if (flip_h == 1 & flip_v == 0):
            image = cv2.flip(image, 1)
        elif (flip_h == 0 & flip_v == 1):
            image = cv2.flip(image, 0)
        elif (flip_h == 1 & flip_v == 1):
            image = cv2.flip(image, -1)
        
        screen.fill([0,0,0])

        # ===== Add red dot here ========
        if task_number == 2:
            image = detector.scan_image(image)
        elif (task_number == 3 or task_number == 4):
            if frame_count % 4 == 0:
                image = detector.scan_image(image)
            else:
                image = detector.scan_partial(image)
        # ===== END add red dot =========

        # ===== Show ORB features =======
        elif task_number == 5:
            detector.show_orb_features(image)
        # ===== END show ORB features ===



        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image = image.swapaxes(0,1)
        image = cv2.flip(image, -1)
        image = pygame.surfarray.make_surface(image)
        screen.blit(image, (0,0))
        pygame.display.update()
        fwd = 0.0
        turn = 0.0
        for event in pygame.event.get():
            if event.type==pygame.QUIT:
                pygame.quit()
                sys.exit()
                cv2.destroyAllWindows()
        key_input = pygame.key.get_pressed()  
        if key_input[pygame.K_LEFT]:
            turn += 1.0
        if key_input[pygame.K_UP]:
            fwd +=1.0
        if key_input[pygame.K_RIGHT]:
            turn -= 1.0
        if key_input[pygame.K_DOWN]:
            fwd -= 1.0
        if key_input[pygame.K_h]:
            if flip_h == 0:
                flip_h = 1
            else:
                flip_h = 0
        if key_input[pygame.K_v]:
            if flip_v == 0:
                flip_v = 1
            else:
                flip_v = 0
        if key_input[pygame.K_q]:
                pygame.quit()
                sys.exit()
                cv2.destroyAllWindows()
                
        command = mbot_motor_pwm_t()
        command.left_motor_pwm =  fwd * FWD_PWM_CMD - turn * TURN_PWM_CMD
        command.right_motor_pwm = fwd * FWD_PWM_CMD + turn * TURN_PWM_CMD
        lc.publish("MBOT_MOTOR_PWM",command.encode())
        rawCapture.truncate(0)


if __name__ == "__main__":
  # Construct the argument parser and parse the arguments
  ap = argparse.ArgumentParser()
  ap.add_argument("-t", "--task", type=int, default=2, help="The task to be attempted")
  args = vars(ap.parse_args())

  main(args["task"])