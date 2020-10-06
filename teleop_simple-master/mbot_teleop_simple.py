import pygame
from pygame.locals import *
from picamera.array import PiRGBArray
from picamera import PiCamera
import matplotlib.pyplot as plt
import cv2
import time
import numpy as np
import sys
sys.path.append("lcmtypes")
import lcm
#from lcmtypes import mbot_motor_pwm_t
from lcmtypes.rpi_state_t import rpi_state_t
from lcmtypes.bbb_state_t import bbb_state_t
from lcmtypes.steer_command_t import steer_command_t
from lcmtypes import turn_command_t
import blue_tape_detectors as btd
import argparse

sys.path.append("vision")
from vision import orb_detector
from orb_detector import ORBDetector

BBB_TURN_STATE = False

def show_pic(Ix):
    plt.figure()
    plt.axis('off')
    plt.imshow(Ix)
    plt.show()

def main(task_number):
    FWD_PWM_CMD = 0.3
    TURN_PWM_CMD = 0.3
    flip_h = 1
    flip_v = 1
    lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=1")
    pygame.init()
    pygame.display.set_caption("MBot TeleOp")
    screen = pygame.display.set_mode([640,480])
    camera = PiCamera()
    camera.resolution = (640, 480)
    camera.framerate = 32
    rawCapture = PiRGBArray(camera, size=(640, 480))
    time.sleep(0.5)


    # ===== Red dot tracker init ============
    cross_detector = ORBDetector(debug=False) 
    cross_img = cv2.imread('cross_ugly.png') # CHANGE ME! - your local photo
    cross_detector.read_cross(cross_img)
    # ===== END red dot tracker init ========



    # ===== Blue object tracker init ========
    detector = btd.Tape_Detector()	
    # ===== END red dot tracker init ========

    # ===== State Machine Init ==============
    state = rpi_state_t()
    state.state = 0
    steer = steer_command_t()
    turn = turn_command_t.turn_command_t()
    # ===== END State Machine Init ==========

    # ===== State Channel Subscription ======
    subscription = lc.subscribe("BBB_STATE", state_handler)
    # ===== END State Channel Subscription ==
    last_p_steer = 0
    last_p_turn = 0
    last_t = time.process_time()
    startTurn = 0
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        if state.state == 1: #wait for state == 1
            lc.handle_timeout(15)
            global BBB_TURN_STATE
            if BBB_TURN_STATE == True:
                state.state = 2 
                startTurn = time.process_time()

        image = frame.array
        # if (flip_h == 1 & flip_v == 0):
        #     image = cv2.flip(image, 1)
        # elif (flip_h == 0 & flip_v == 1):
        #     image = cv2.flip(image, 0)
        # elif (flip_h == 1 & flip_v == 1):
        #     image = cv2.flip(image, -1)
        #show_pic(cv2.flip(image,0)) #horizontal
        image = cv2.flip(image,1) # CHANGE ME! -1 = vertical flip
        #show_pic(cv2.flip(image,-1))#both
        screen.fill([0,0,0])

        if state.state == 0:
            #start = time.time()	
            # ===== Blue line detection =====        
            found, center = detector.search_stopline(image)	
            if found:		
                found, center = detector.search_stopline(image)	
                if found:
                    print("CENTER:", center)	
                    image = cv2.circle(image, center, 15, (0,255,0), -1)	
                    state.state = 1
                    lc.publish("RPI_STATE",state.encode())
            # ===== END Blue line detection ======
            
            # ===== Add red dot here ========
            if not found:
                _, center = cross_detector.show_orb_features(image)
                if center is not None:
                    steer.p_term = center[0] - (image.shape[1] // 2)
                    cur_t = time.time()
                    delta_t = (cur_t - last_t)
                    steer.d_term = (steer.p_term - last_p) // (delta_t)
                    last_p = steer.p_term
                    last_t = time.process_time()

                    print("STEER COMMAND: p = " , steer.p_term, " d = ", steer.d_term, "delta_time = ", cur_t-last_t)
                    last_t = cur_t
                    lc.publish("STEER",steer.encode())
                lc.publish("RPI_STATE",state.encode())
            # ===== END add red dot =========
            #print("time elapsed:", time.time() - start)

        elif state.state == 1:
            lc.publish("RPI_STATE",state.encode())
            '''
            Do we want to pass some kind of information to the stopping functionality?
            '''
        elif state.state == 2:
            BBB_TURN_STATE = False
            timePassed = time.process_time() - startTurn
            if(timePassed > 1.5 ):
                state.state = 0

            found, center = detector.search_stopline(image)
            if found is not None:
                turn.p_term = center[0] - 570 # TODO -> Parameterize
                delta_t = time.process_time() - last_t
                turn.d_term = (turn.p_term - last_p_turn) // (last_t)
                last_p_turn = turn.p_term
                last_t = time.process_time()
            else: 
                pass
                #TODO -> HOW TO HANDLE THIS
        else:
            print("STATE ERROR")


        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image = image.swapaxes(0,1)
        #image = cv2.flip(image, -1)
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

        rawCapture.truncate(0)

def state_handler(channel, data):
    msg = bbb_state_t().decode(data)
    print("Received message: ", msg.state, " on channel ", channel)
    if msg.state == 2:
        global BBB_TURN_STATE
        BBB_TURN_STATE = True

if __name__ == "__main__":
  # Construct the argument parser and parse the arguments
  ap = argparse.ArgumentParser()
  ap.add_argument("-t", "--task", type=int, default=2, help="The task to be attempted")
  args = vars(ap.parse_args())

  main(args["task"])


