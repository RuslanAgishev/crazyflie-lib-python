#!/usr/bin/env python

# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2016 Bitcraze AB
#
#  Crazyflie Nano Quadcopter Client
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#  MA  02110-1301, USA.
"""
Simple example that connects to one crazyflie (check the address at the top
and update it to your crazyflie address) and send a sequence of setpoints,
one every 5 seconds.

This example is intended to work with the Loco Positioning System in TWR TOA
mode. It aims at documenting how to set the Crazyflie in position control mode
and how to send setpoints.
"""
import threading
import time
import numpy as np
from numpy.linalg import norm

import cflib.crtp
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie import Crazyflie

from tools import reset_estimator
from drone import Drone
import pandas as pd

def trajectory_minjerk(drone, scale=1.0):
    """ Min-jerk trajectory from a CSV-file """
    traj = pd.read_csv('minjerk_trajectory.csv')
    traj = np.array(traj)
    drone.sp = np.zeros(4)
    drone.sp[:3] = drone.pose
    traj += drone.pose 
    for i in range(traj.shape[0]):
        drone.sp[:3] = traj[i,:] * scale
        drone.fly()
        time.sleep(0.01)

def connect(drone):
    with SyncCrazyflie(drone.uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        drone.scf = scf
        reset_estimator(drone)

# URI to the Crazyflie to connect to
URI1 = 'radio://0/80/2M/E7E7E7E703'
URI2 = 'radio://0/80/2M/E7E7E7E703'

waypoints1 = [
    np.array([0.5, 0.0, 0.3, 0]),
    np.array([0.5, 0.0, 1.3, 0]),
    np.array([0.0, 0.0, 0.3, 0]),
]
waypoints2 = [
    np.array([0.0, -0.5, 0.3, 0]),
    np.array([0.5, -0.5, 0.6, 0]),
    np.array([-0.5, 0.5, 0.3, 0]),
]

if __name__ == '__main__':
    cflib.crtp.init_drivers(enable_debug_driver=False)
    drone1 = Drone(URI1)
    drone2 = Drone(URI2)

    threading.Thread(target=connect, args=(drone1,)).start()
    # threading.Thread(target=connect, args=(drone2,)).start()
    time.sleep(2)

    """ 1st mission """
    drone1.start_position_reading() # 20 Hz
    drone1.start_battery_status_reading() # 2 Hz
    time.sleep(1)
    drone1.pose_home = drone1.pose

    # drone1.takeoff(height=0.3)
    # drone1.hover(1)

    # # for goal in waypoints1:
    # #     drone1.goTo(goal)
    # #     drone1.hover(1)

    # drone1.goTo([drone1.pose_home[0], drone1.pose_home[1], 0.3, 0])
    # drone1.hover(2)

    # drone1.land()


    """ 2nd mission """
    # drone2.start_position_reading() # 20 Hz
    # drone2.start_battery_status_reading() # 2 Hz
    # time.sleep(1)
    # drone2.pose_home = drone2.pose

    # # trajectory_minjerk(drone2, scale=0.3)

    # # for waypoint in waypoints2:
    # #     drone2.goTo(waypoint)
    # #     drone2.hover(1)

    # drone2.goTo([drone2.pose_home[0], drone2.pose_home[1], 0.3, 0])
    # drone2.hover(2)

    # drone2.land()

