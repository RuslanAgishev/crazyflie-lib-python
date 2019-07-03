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
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger


class Drone(object):
    """docstring for Drone"""
    def __init__(self, uri='radio://0/80/2M'):
        super(Drone, self).__init__()
        self.uri = uri # URI to the Crazyflie to connect to
        self.cf = SyncCrazyflie(self.uri, cf=Crazyflie(rw_cache='./cache')).cf
        self.pose = None
        self.sp = None

    def fly(self):
        self.cf.commander.send_position_setpoint(self.sp[0], self.sp[1], self.sp[2], self.sp[3])

    def takeoff(self):
        # takeoff to z=0.5 m:
        print('Takeoff...')
        self.sp = np.zeros(4); self.sp[:3] = self.pose
        for i in range(50):
            self.sp[2] += 0.01
            self.fly()
            time.sleep(0.1)

    def land(self):
        print('Landing...')
        while self.sp[2]>-0.1:
            self.sp[2] -= 0.03
            self.fly()
            time.sleep(0.1)

        self.stop()
        # Make sure that the last packet leaves before the link is closed
        # since the message queue is not flushed before closing
        time.sleep(0.1)
    def stop(self):
        self.cf.commander.send_stop_setpoint()

    def goTo(self, goal, pos_tol=0.03, yaw_tol=3):
        goal = np.array(goal)
        print('Going to', goal)
        while norm(goal[:3] - self.sp[:3]) > pos_tol or norm(self.sp[3]-goal[3]) > yaw_tol:
            n = normalize(goal[:3] - self.sp[:3])
            self.sp[:3] += 0.03 * n # position setpoints
            self.sp[3] += 3 * np.sign( goal[3] - self.sp[3] ) # yaw angle
            print('Yaw', self.sp[3], 'yaw diff', norm(self.sp[3]-goal[3]))
            self.fly()
            time.sleep(0.1)

    def position_callback(self, timestamp, data, logconf):
        x = data['kalman.stateX']
        y = data['kalman.stateY']
        z = data['kalman.stateZ']
        self.pose = np.array([x,y,z])
    def start_position_printing(self):
        log_conf = LogConfig(name='Position', period_in_ms=50)
        log_conf.add_variable('kalman.stateX', 'float')
        log_conf.add_variable('kalman.stateY', 'float')
        log_conf.add_variable('kalman.stateZ', 'float')

        self.cf.log.add_config(log_conf)
        log_conf.data_received_cb.add_callback(self.position_callback)
        log_conf.start()



def normalize(vector):
    vector = np.array(vector)
    v_norm = vector / norm(vector) if norm(vector)!=0 else np.zeros_like(vector)
    return v_norm