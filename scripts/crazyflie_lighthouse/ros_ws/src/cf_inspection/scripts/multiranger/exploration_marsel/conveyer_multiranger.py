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

from tools import reset_estimator
from drone import Drone

import rospy

from multiranger_navigation import Mission

# URI to the Crazyflie to connect to
uri1 = 'radio://0/80/2M/E7E7E7E701'
uri2 = 'radio://0/80/2M/E7E7E7E703'


def execute_mission(mission, height):
    mission.coverage_mission(height=height, length=1.5, width=0.3, numiters=3)


if __name__ == '__main__':
    rospy.init_node('conveyer')
    cflib.crtp.init_drivers(enable_debug_driver=False)

    mission1 = Mission(uri1)
    mission2 = Mission(uri2)

    th1 = Thread(target=execute_mission, args=(mission1, 0.3) )
    th2 = Thread(target=execute_mission, args=(mission2, 1.0) )

    th1.start(); th2.start()
    th1.join(); th2.join()
