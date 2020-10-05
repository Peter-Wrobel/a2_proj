import sys
sys.path.append("lcmtypes")
import lcm
#from lcmtypes import mbot_motor_pwm_t
from lcmtypes import state_t
import argparse

lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=1")

def handler(channel,data):
    msg = state_t.state_t().decode(data)
    print(msg.state)

subscribe = lc.subscribe("STATE",handler)

try:
    while True:
            lc.handle()
except KeyboardInterrupt:
    pass