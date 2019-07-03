
import logging
import time
import numpy as np

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

URI = 'radio://0/80/2M'

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)


if __name__ == '__main__':
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        cf = scf.cf

        cf.param.set_value('kalman.resetEstimation', '1')
        time.sleep(0.1)
        cf.param.set_value('kalman.resetEstimation', '0')
        time.sleep(2)

        # HEIGHT = 1.3
        # # takeoff
        # for y in range(65):
        #     cf.commander.send_hover_setpoint(0, 0, 0, y / 50)
        #     time.sleep(0.1)
        print('Takeoff...')
        sp = np.zeros(4)
        for i in range(50):
            sp[2] += 0.026
            cf.commander.send_position_setpoint(sp[0], sp[1], sp[2], sp[3])
            time.sleep(0.1)

        # Figure 8
        # for _ in range(20):
        #     cf.commander.send_hover_setpoint(0, 0, 0, HEIGHT)
        #     time.sleep(0.1)

        # for _ in range(50):
        #     cf.commander.send_hover_setpoint(0.5, 0, 36 * 2, HEIGHT)
        #     time.sleep(0.1)

        # for _ in range(50):
        #     cf.commander.send_hover_setpoint(0.5, 0, -36 * 2, HEIGHT)
        #     time.sleep(0.1)

        # for _ in range(20):
        #     cf.commander.send_hover_setpoint(0, 0, 0, HEIGHT)
        #     time.sleep(0.1)

        # land
        for y in range(65):
            cf.commander.send_hover_setpoint(0, 0, 0, (65 - y) / 50)
            time.sleep(0.1)

        cf.commander.send_stop_setpoint()
