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
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie import Crazyflie
from cflib.positioning.motion_commander import MotionCommander

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




if __name__ == '__main__':
    rospy.init_node('mission_swarm_pointcloud')
    cflib.crtp.init_drivers(enable_debug_driver=False)

    drone1 = DroneMultiranger(URI1)
    drone2 = DroneMultiranger(URI2)

    th1 = Thread(target=flight, args=(drone1,) )
    th2 = Thread(target=flight, args=(drone2, 'romb') )

    th1.start(); th2.start()
    th1.join(); th2.join()

