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

# URI to the Crazyflie to connect to
uri = 'radio://0/80/2M'
toFly = 0

# Change the sequence according to your setup
#             x    y    z  YAW
# sequence = [
#     np.array([0.0, 0.0, 0.5, 0]),
#     np.array([0.3, 0.0, 0.5, 90]),
#     np.array([0.0, -0.3, 0.5, 180]),
# ]

sequence = [
    np.array([0.0, 0.0, 1.3, 0]),
    np.array([0.8, -0.6, 1.3, 90]),
    np.array([-0.8, 0.4, 1.3, 180]),
    np.array([-0.8, 0.0, 1.3, 0]),
    np.array([0.5, 0.2, 1.3, 0]),
    np.array([0.0, 0.0, 0.1, 0]),
]


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


def reset_estimator(scf):
    cf = scf.cf
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')

    wait_for_position_estimator(cf)


def position_callback(timestamp, data, logconf):
    x = data['kalman.stateX']
    y = data['kalman.stateY']
    z = data['kalman.stateZ']
    print('pos: ({}, {}, {})'.format(x, y, z))


def start_position_printing(scf):
    log_conf = LogConfig(name='Position', period_in_ms=500)
    log_conf.add_variable('kalman.stateX', 'float')
    log_conf.add_variable('kalman.stateY', 'float')
    log_conf.add_variable('kalman.stateZ', 'float')

    scf.cf.log.add_config(log_conf)
    log_conf.data_received_cb.add_callback(position_callback)
    log_conf.start()

def normalize(vector):
    vector = np.array(vector)
    v_norm = vector / norm(vector) if norm(vector)!=0 else np.zeros_like(vector)
    return v_norm

def run(scf, sequence):
    cf = scf.cf

    # takeoff from [0,0,0,0] to z=0.5 m:
    print('Takeoff...')
    sp = np.zeros(4)
    for i in range(50):
        sp[2] += 0.01
        if toFly: cf.commander.send_position_setpoint(sp[0], sp[1], sp[2], sp[3])
        time.sleep(0.1)

    # waypoints sequence
    pos_tol = 0.03; yaw_tol = 3
    for goal in sequence:
        print('Going to', goal)
        while norm(goal[:3] - sp[:3]) > pos_tol or norm(sp[3]-goal[3]) > yaw_tol:
            n = normalize(goal[:3] - sp[:3])
            sp[:3] += 0.03 * n # position setpoints
            sp[3] += 3 * np.sign( goal[3] - sp[3] ) # yaw angle
            # print('Yaw', sp[3], 'yaw diff', norm(sp[3]-goal[3]))
            if toFly: cf.commander.send_position_setpoint(sp[0], sp[1], sp[2], sp[3])
            time.sleep(0.1)

        # hold position for 2 sec
        t0 = time.time()
        while time.time() - t0 < 2.0:
            if toFly: cf.commander.send_position_setpoint(sp[0], sp[1], sp[2], sp[3])
            time.sleep(0.1)


    # land:
    print('Landing...')
    while sp[2]>-0.1:
        print(sp[2])
        sp[2] -= 0.03
        if toFly: cf.commander.send_position_setpoint(sp[0], sp[1], sp[2], sp[3])
        time.sleep(0.1)

    if toFly: cf.commander.send_stop_setpoint()
    # Make sure that the last packet leaves before the link is closed
    # since the message queue is not flushed before closing
    time.sleep(0.1)


if __name__ == '__main__':
    cflib.crtp.init_drivers(enable_debug_driver=False)

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        reset_estimator(scf)
        start_position_printing(scf)
        run(scf, sequence)
