import math
import matplotlib.pyplot as plt
import numpy as np
from grid_based_sweep_coverage_path_planner import planning
from connect_crazyflie import reset_estimator
from connect_crazyflie import activate_high_level_commander
from connect_crazyflie import activate_mellinger_controller

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
import time

def planning_animation(ox, oy, reso):  # pragma: no cover
    px, py = planning(ox, oy, reso)

    # animation
    if do_animation:
        for ipx, ipy in zip(px, py):
            plt.cla()
            plt.plot(ox, oy, "-xb")
            plt.plot(px, py, "-r")
            plt.plot(ipx, ipy, "or")
            plt.axis("equal")
            plt.grid(True)
            plt.pause(0.1)

    plt.cla()
    plt.plot(ox, oy, "-xb")
    plt.plot(px, py, "-r")
    plt.axis("equal")
    plt.grid(True)
    plt.pause(0.1)

    return np.array(px), np.array(py)
    

def run_sequence(scf, sequence):
    commander = scf.cf.commander
    hl_commander = scf.cf.high_level_commander

    if TO_FLY:
        print('Takeoff...')
        hl_commander.takeoff(0.3, 3.0)
        time.sleep(3)
        # Going to initial locations
        flight_time = 3.0
        hl_commander.go_to(sequence[0,0], sequence[0,1], sequence[0,2], sequence[0,3],
                           flight_time, relative=False)
        time.sleep(flight_time)
        hl_commander.stop()
        time.sleep(0.01)

    for sp in sequence:
        if TO_FLY: commander.send_position_setpoint(sp[0], sp[1], sp[2], sp[3])
        time.sleep(0.2)

    # Landing...
    while sp[2] > -0.1:
        sp[2] -= 0.02
        if TO_FLY: commander.send_position_setpoint(sp[0], sp[1], sp[2], sp[3])
        time.sleep(0.1)

    if TO_FLY: commander.send_stop_setpoint()
    # Make sure that the last packet leaves before the link is closed
    # since the message queue is not flushed before closing
    time.sleep(0.1)


do_animation = 0
TO_FLY = 1
height = 0.5
URI = 'radio://0/80/2M/E7E7E7E703'

if __name__ == '__main__':

    cflib.crtp.init_drivers(enable_debug_driver=False)
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
	    if TO_FLY:
	        activate_high_level_commander(scf)
	        reset_estimator(scf)
	        activate_mellinger_controller(scf, False)

	    SCALE = 1.0

	    ox = np.array([-1.0, 1.0,  1.0, -1.0, -1.0]) * SCALE
	    oy = np.array([1.0,  1.0, -1.0, -1.0,  1.0]) * SCALE
	    reso = 0.1
	    
	    px, py = planning_animation(ox, oy, reso)

	    px = px[np.newaxis].T; py = py[np.newaxis].T # making column vectors to stuck in sequence
	    sequence = np.hstack([px, py, height*np.ones_like(px), np.zeros_like(px)])

	    plt.pause(0.1)
	    input('Hit Enter to fly')
	    plt.close('all')

	    print("Flight!!")
	    run_sequence(scf, sequence)

