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
Simple example that connects to crazyflies one by one (check the addresses at the top
and update it to your crazyflies addresses) and sends a mission: sequence of setpoints
or figure8 trajectory.

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

from drone import Drone, reset_estimator
import pandas as pd


# URI to the Crazyflie to connect to
URI1 = 'radio://0/80/2M/E7E7E7E701'
URI2 = 'radio://0/80/2M/E7E7E7E702'
URI3 = 'radio://0/80/2M/E7E7E7E703'

def connect(drone):
    with SyncCrazyflie(drone.uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        drone.scf = scf
        reset_estimator(drone)
        drone.start_position_reading() # 20 Hz
        drone.start_battery_status_reading() # 2 Hz
        time.sleep(1)
        drone.pose_home = drone.pose

def trajectory_minjerk(drone, scale=1.0):
    """ Min-jerk trajectory from a CSV-file """
    traj = pd.read_csv('minjerk_trajectory.csv')
    traj = np.array(traj)
    drone.sp = drone.pose
    traj += drone.pose 
    for i in range(traj.shape[0]):
        drone.sp[:3] = traj[i,:] * scale
        drone.fly()
        time.sleep(0.1)


if __name__ == '__main__':
    rospy.init_node('cf_control')
    cflib.crtp.init_drivers(enable_debug_driver=False)
    drone1 = Drone(URI1)
    drone2 = Drone(URI2)
    drone3 = Drone(URI3)

    # Connecting to all drones in the swarm before flights.
    # During this time each drone estimates its own position.
    threading.Thread(target=connect, args=(drone1,)).start()
    threading.Thread(target=connect, args=(drone2,)).start()
    threading.Thread(target=connect, args=(drone3,)).start()

    
    """ First mission """
    with SyncCrazyflie(drone1.uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        print('Home position:', drone1.pose_home)
        print('Battery status %.2f:' %drone1.V_bat)

        # take a look at drone.py line 47: V_BATTERY_CHARGED = 3.85
        if drone1.battery_state == 'fully_charged':
            print('drone1 is ready for takeoff')
            drone1.takeoff(height=0.3)
            drone1.hover(1)

            """ Flight mission """
            # Specify here waypoint to follow in the format:
            # wp = [wp_x, wp_y, wp_z, wp_yaw], where coordinates are given in global
            # coordinate frame, relative to the origin, [0,0,0,0].
            #                X     Y     Z    YAW
            wp_sequence = [
                           [ 0.8, -0.6,  1.3, 90],
                           [-0.8,  0.4,  1.3, 180],
                           [-0.8,  0.0,  1.3, 0],
                           [ 0.5,  0.2,  1.3, 0],
                           [ 0.0, -0.4,  1.3, 0],
                          ]

            for waypoint in wp_sequence:
                if not drone1.battery_state == 'needs_charging':
                    drone1.goTo(waypoint)
                    drone1.hover(1)

            print('Go home before landing...')
            drone1.goTo([drone1.pose_home[0], drone1.pose_home[1], 0.3, 0])
            drone1.hover(2)
            drone1.land()
            print('Battery status: %.2f [V]' %drone1.V_bat)


    """ Second mission """
    with SyncCrazyflie(drone2.uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        print('Home position:', drone2.pose_home)
        print('Battery status %.2f:' %drone2.V_bat)

        # take a look at drone.py line 47: V_BATTERY_CHARGED = 3.85
        if drone2.battery_state == 'fully_charged':
            drone2.takeoff(height=0.3)
            drone2.hover(1)

            # Going to the position convenient to execute a trajectory
            drone2.goTo([0.0, -0.3, 1.3, 0])
            drone2.hover(2)

            drone2.trajectory_figure8_battery_check()

            print('Go home before landing...')
            drone2.goTo([drone2.pose_home[0], drone2.pose_home[1], 0.3, 0])
            drone2.hover(2)
            drone2.land()
            print('Battery status: %.2f [V]' %drone2.V_bat)



    """ Fird mission """
    with SyncCrazyflie(drone3.uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        print('Home position:', drone3.pose_home)
        print('Battery status %.2f:' %drone3.V_bat)

        # take a look at drone.py line 47: V_BATTERY_CHARGED = 3.85
        if drone3.battery_state == 'fully_charged':
            drone3.takeoff(height=0.3)
            drone3.hover(1)

            # Going to the position convenient to execute a trajectory
            drone3.goTo([0.0, -0.3, 1.3, 0])
            drone3.hover(2)

            trajectory_minjerk(drone3, scale=0.66)

            drone3.land()
            print('Battery status: %.2f [V]' %drone3.V_bat)

