# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2019 Bitcraze AB
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
Simple example of a swarm using the High level commander.

The swarm takes off and flies a synchronous square shape before landing.
The trajectories are relative to the starting positions and the Crazyfles can
be at any position on the floor when the script is started.

This example is intended to work with any absolute positioning system.
It aims at documenting how to use the High Level Commander together with
the Swarm class.
"""
import time
import numpy as np

import cflib.crtp
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm
from cflib.crazyflie.syncLogger import SyncLogger


def position_callback(timestamp, data, logconf):
    x = data['kalman.stateX']
    y = data['kalman.stateY']
    z = data['kalman.stateZ']
    pose = np.array([x, y, z])
    print(pose)
def start_position_reading(scf):
    log_conf = LogConfig(name='Position', period_in_ms=50) # read position with 20 Hz rate
    log_conf.add_variable('kalman.stateX', 'float')
    log_conf.add_variable('kalman.stateY', 'float')
    log_conf.add_variable('kalman.stateZ', 'float')

    scf.cf.log.add_config(log_conf)
    log_conf.data_received_cb.add_callback(position_callback)
    log_conf.start()


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
    wait_for_position_estimator(scf)


def activate_high_level_commander(scf):
    scf.cf.param.set_value('commander.enHighLevel', '1')


def activate_mellinger_controller(scf, use_mellinger):
    controller = 1
    if use_mellinger:
        controller = 2
    scf.cf.param.set_value('stabilizer.controller', controller)


def run_shared_sequence(scf, waypoints):
    activate_mellinger_controller(scf, False)

    flight_time = 4

    commander = scf.cf.high_level_commander

    commander.takeoff(0.3, 2.0)
    time.sleep(3)

    for goal in waypoints:
        commander.go_to(goal[0], goal[1], goal[2], goal[3]/180*3.14, flight_time, relative=False)
        time.sleep(flight_time)

    # commander.go_to(0, 0, 0.2, 0.0, flight_time, relative=False)
    # time.sleep(flight_time)

    commander.land(0.0, 5.0)
    time.sleep(2)
    commander.stop()


URI1 = 'radio://0/80/2M/E7E7E7E701'
URI2 = 'radio://0/80/2M/E7E7E7E702'
URI3 = 'radio://0/80/2M/E7E7E7E703'

uris = {
    # URI1,
    URI2,
    # URI3,
}

# x[m], y[m], z[m], yaw[deg]
waypoints1 = [
            (0.0, 0.7, 1.2, 0),
            (0.7, 0.0, 1.3, -90),
            (0.0, -0.7, 1.5, -180),
            (-0.7, 0.0, 1.4, -270),
            (0.0, 0.7, 1.2, 0),
            (0.7, 0.0, 1.3, -90),
            (0.0, -0.7, 1.5, -180),
            (-0.7, 0.0, 1.4, -270),

            (0.0, 0.3, 0.3, 0),
            ]
# x[m], y[m], z[m], yaw[deg]
waypoints2 = [
            (0.0, 0.0, 1.5, 0), 
            (0.0, 0.0, 1.3, -90),
            (0.0, 0.0, 1.3, -180 ),
            (0.0, 0.0, 1.2, -90),
            (0.0, 0.0, 1.6, 0 ),
            (0.0, 0.0, 1.6, -90),
            (0.0, 0.0, 1.6, -180 ),
            (0.0, 0.0, 1.6, -90),

            (0.0, 0.0, 0.0, 0),
            ]
# x[m], y[m], z[m], yaw[deg]
waypoints3 = [
            (0.0, -0.7, 1.2, 0),
            (-0.7, 0.0, 1.4, -90),
            (0.0, 0.7, 1.5, -180),
            (0.7, 0.0, 1.2, -270),
            (0.0, -0.7, 1.2, 0),
            (-0.7, 0.0, 1.4, -90),
            (0.0, 0.7, 1.5, -180),
            (0.7, 0.0, 1.2, -270),

            (0.0, -0.5, 0.3, 0),
            ]

wp_args = {
    # URI1: [waypoints1],
    URI2: [waypoints2],
    # URI3: [waypoints3],
}

if __name__ == '__main__':
    cflib.crtp.init_drivers(enable_debug_driver=False)
    factory = CachedCfFactory(rw_cache='./cache')
    with Swarm(uris, factory=factory) as swarm:
        swarm.parallel_safe(activate_high_level_commander)
        swarm.parallel_safe(reset_estimator)
        swarm.parallel_safe(start_position_reading)
        swarm.parallel_safe(run_shared_sequence, args_dict=wp_args)
