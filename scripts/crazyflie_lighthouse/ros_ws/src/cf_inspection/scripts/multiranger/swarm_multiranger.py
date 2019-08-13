#!/usr/bin/env python

# -*- coding: utf-8 -*-

"""
Simple example that connects to one crazyflie (check the address at the top
and update it to your crazyflie address) and send a sequence of setpoints,
one every 5 seconds.

This example is intended to work with the Loco Positioning System in TWR TOA
mode. It aims at documenting how to set the Crazyflie in position control mode
and how to send setpoints.
"""

import time
import numpy as np
from numpy.linalg import norm
from threading import Thread

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm
from cflib.crazyflie.syncLogger import SyncLogger


import rospy

from multiranger import DroneMultiranger

# URI to the Crazyflie to connect to
URI1 = 'radio://0/80/2M/E7E7E7E702'
URI2 = 'radio://0/80/2M/E7E7E7E703'


def flight(drone, label='', numiters=1):
    drone.mc = MotionCommander(drone.cf)
    drone.mc.take_off(0.3, 0.2)
    time.sleep(1)

    if label=='romb':
        drone.mc.turn_left(45)
        time.sleep(1)
    # sequence of repeated squares
    for _ in range(numiters):
        # sqaure
        for _ in range(4):
            drone.mc.forward(1.0)
            time.sleep(2)
            drone.mc.turn_right(90)
            time.sleep(1)

    drone.mc.land(0.2)
    time.sleep(1)



def wait_for_position_estimator(scf):
    print('Waiting for estimator to find position...')

    log_config = LogConfig(name='Kalman Variance', period_in_ms=500)
    log_config.add_variable('kalman.varPX', 'float')
    log_config.add_variable('kalman.varPY', 'float')
    log_config.add_variable('kalman.varPZ', 'float')

    var_y_history = [1000] * 10
    var_x_history = [1000] * 10
    var_z_history = [1000] * 10

    threshold = 0.001

    with SyncLogger(scf, log_config) as logger:
        for log_entry in logger:
            data = log_entry[1]

            var_x_history.append(data['kalman.varPX'])
            var_x_history.pop(0)
            var_y_history.append(data['kalman.varPY'])
            var_y_history.pop(0)
            var_z_history.append(data['kalman.varPZ'])
            var_z_history.pop(0)

            min_x = min(var_x_history)
            max_x = max(var_x_history)
            min_y = min(var_y_history)
            max_y = max(var_y_history)
            min_z = min(var_z_history)
            max_z = max(var_z_history)

            # print("{} {} {}".
            #       format(max_x - min_x, max_y - min_y, max_z - min_z))

            if (max_x - min_x) < threshold and (
                    max_y - min_y) < threshold and (
                    max_z - min_z) < threshold:
                break


def reset_estimator(cf):
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')
    wait_for_position_estimator(scf)


def activate_high_level_commander(cf):
    cf.param.set_value('commander.enHighLevel', '1')


def activate_mellinger_controller(cf, use_mellinger):
    controller = 1
    if use_mellinger:
        controller = 2
    cf.param.set_value('stabilizer.controller', controller)


def run_shared_sequence(scf):
    activate_mellinger_controller(scf, False)

    box_size = 1
    flight_time = 2

    commander = scf.cf.high_level_commander

    commander.takeoff(1.0, 2.0)
    time.sleep(3)

    commander.go_to(box_size, 0, 0, 0, flight_time, relative=True)
    time.sleep(flight_time)

    commander.go_to(0, box_size, 0, 0, flight_time, relative=True)
    time.sleep(flight_time)

    commander.go_to(-box_size, 0, 0, 0, flight_time, relative=True)
    time.sleep(flight_time)

    commander.go_to(0, -box_size, 0, 0, flight_time, relative=True)
    time.sleep(flight_time)

    commander.land(0.0, 2.0)
    time.sleep(2)

    commander.stop()


uris = {
    URI1,
    URI2,
}

if __name__ == '__main__':
    rospy.init_node('mission_swarm_pointcloud')
    cflib.crtp.init_drivers(enable_debug_driver=False)
    factory = CachedCfFactory(rw_cache='./cache')
    with Swarm(uris, factory=factory) as swarm:
        swarm.parallel_safe(activate_high_level_commander)
        swarm.parallel_safe(reset_estimator)

        drone1 = DroneMultiranger(URI1)
        drone2 = DroneMultiranger(URI2)

#         swarm.parallel_safe(run_shared_sequence)


if __name__ == '__main__':
    rospy.init_node('mission_swarm_pointcloud')
    cflib.crtp.init_drivers(enable_debug_driver=False)

    drone1 = DroneMultiranger(URI1)
    drone2 = DroneMultiranger(URI2)

    for cf in [drone1.cf, drone2.cf]:
        activate_high_level_commander(cf)
        reset_estimator(cf)

    th1 = Thread(target=flight, args=(drone1,) )
    th2 = Thread(target=flight, args=(drone2, 'romb') )

    th1.start(); th2.start()
    th1.join(); th2.join()

