
"""
The MotionCommander uses velocity setpoints.
"""
import logging
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander

URI = 'radio://0/80/2M'

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)


if __name__ == '__main__':
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        # We take off when the commander is created
        with MotionCommander(scf) as mc:
            time.sleep(1)

            # There is a set of functions that move a specific distance
            # We can move in all directions
            mc.up(0.5)
            time.sleep(1)
            
            mc.turn_left(180)
            time.sleep(1)

            mc.down(0.5)
            time.sleep(1)

            mc.turn_left(180)
            time.sleep(1)
            

            
            # for _ in range(5):
            #     print('Doing other work')
            #     time.sleep(0.2)

            # And we can stop
            mc.stop()

            # We land when the MotionCommander goes out of scope
