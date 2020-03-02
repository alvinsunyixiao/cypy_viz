import math
import time

import lcm

from comm import pose3d_t

RADIUS      = 4      # in [m]
DELTA_T     = 0.05   # in [s]
OMEGA       = 1      # in [rad / s]
NUM_ROBOT   = 4

if __name__ == '__main__':
    # global lcm handle
    lc = lcm.LCM()
    msg = pose3d_t()
    # angle accumulator
    theta = 0
    # event loop
    while True:
        for i in range(NUM_ROBOT):
            theta_i = theta if i % 2 else 2 * math.pi - theta
            msg.position.x = RADIUS * (i+1) / NUM_ROBOT * math.cos(theta_i)
            msg.position.y = RADIUS * (i+1) / NUM_ROBOT * math.sin(theta_i)
            msg.rotation.yaw = theta_i + math.pi / 2 if i % 2 else theta_i - math.pi / 2
            lc.publish('f1tenth_clone_{}'.format(i), msg.encode())
        theta += OMEGA * DELTA_T
        time.sleep(DELTA_T)
