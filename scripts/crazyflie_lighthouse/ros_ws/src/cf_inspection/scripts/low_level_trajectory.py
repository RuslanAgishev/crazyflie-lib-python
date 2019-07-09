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
import time
import numpy as np
from numpy.linalg import norm

import cflib.crtp
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie import Crazyflie

from tools import reset_estimator
from drone import Drone

import rospy


def normalize(vector):
    vector = np.array(vector)
    v_norm = vector / norm(vector) if norm(vector)!=0 else np.zeros_like(vector)
    return v_norm

# URI to the Crazyflie to connect to
uri = 'radio://0/80/2M/E7E7E7E702'
toFly = 0


if __name__ == '__main__':
    rospy.init_node('cf_control')
    cflib.crtp.init_drivers(enable_debug_driver=False)
    drone = Drone(uri)

    with SyncCrazyflie(drone.uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        drone.scf = scf
        reset_estimator(drone)
        drone.start_position_reading() # 20 Hz
        drone.start_battery_status_reading() # 2 Hz
        time.sleep(1)

        drone.pose_home = drone.pose
        print('Home position:', drone.pose_home)
        print('Battery status: %.2f' %drone.V_bat)


        # takeoff to z=0.3 m:
        height = 0.3
        print('Takeoff...')
        drone.sp = np.zeros(4); drone.sp[:3] = drone.pose
        dz = 0.02
        for i in range(int(height/dz)):
            drone.sp[2] += dz
            if toFly: drone.fly()
            drone.publish_sp()
            drone.publish_path() if toFly else drone.publish_path_sp()
            time.sleep(0.1)

        # """ Flight mission """
        goal = np.array([0.5, 0.0, 0.3, 0])
        pos_tol = 0.03; yaw_tol = 3
        print('Going to', goal)
        while norm(goal[:3] - drone.sp[:3]) > pos_tol or norm(drone.sp[3]-goal[3]) > yaw_tol:
            n = normalize(goal[:3] - drone.sp[:3])
            drone.sp[:3] += 0.03 * n # position setpoints
            drone.sp[3] += 3 * np.sign( goal[3] - drone.sp[3] ) # yaw angle
            if toFly: drone.fly()
            drone.publish_sp()
            drone.publish_path() if toFly else drone.publish_path_sp()
            time.sleep(0.1)


        print('Landing...')
        while drone.sp[2]>-0.1:
            drone.sp[2] -= 0.02
            if toFly: drone.fly()
            drone.publish_sp()
            drone.publish_path() if toFly else drone.publish_path_sp()
            time.sleep(0.1)
        drone.cf.commander.send_stop_setpoint()
        # Make sure that the last packet leaves before the link is closed
        # since the message queue is not flushed before closing
        time.sleep(0.1)
        
        print('Battery status %.2f:' %drone.V_bat)

