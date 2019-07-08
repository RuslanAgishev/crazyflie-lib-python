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


# URI to the Crazyflie to connect to
uri = 'radio://0/80/2M/E7E7E7E702'

if __name__ == '__main__':
    cflib.crtp.init_drivers(enable_debug_driver=False)
    drone = Drone(uri)

    """ First mission """
    with SyncCrazyflie(drone.uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        drone.scf = scf
        reset_estimator(drone)
        drone.start_position_reading() # 20 Hz
        drone.start_battery_status_reading() # 2 Hz
        time.sleep(1)

        drone.pose_home = drone.pose
        print('Home position:', drone.pose_home)
        print('Battery status %.2f:' %drone.V_bat)

        if drone.battery_state == 'fully_charged':
            print('Drone is ready for takeoff')
            drone.takeoff(height=0.3)
            drone.hover(1)

            """ Flight mission """
            wp_sequence = [
                           [0.3, 0.0, 0.3, 90],
                           [0.3, -0.3, 0.3, 0],
                          ]

            # wp_sequence = [
            #                [0.8, -0.6, 1.3, 90],
            #                [-0.8, 0.4, 1.3, 180],
            #                [-0.8, 0.0, 1.3, 0],
            #                [0.5, 0.2, 1.3, 0],
            #                [0.0, -0.4, 1.3, 0],
            #               ]

            for waypoint in wp_sequence:
                if not drone.battery_state == 'needs_charging':
                    drone.goTo(waypoint)
                    drone.hover(1)

            print('Go home before landing...')
            drone.goTo([drone.pose_home[0], drone.pose_home[1], 0.3, 0])
            drone.hover(2)
            drone.land()
            print('Battery status: %.2f [V]' %drone.V_bat)


    """ Second mission """
    with SyncCrazyflie(drone.uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        drone.scf = scf
        reset_estimator(drone)
        drone.start_position_reading()
        drone.start_battery_status_reading() # 2 Hz
        time.sleep(1)

        drone.pose_home = drone.pose
        print('Home position:', drone.pose_home)
        print('Battery status %.2f:' %drone.V_bat)

        if drone.battery_state == 'fully_charged':
            drone.takeoff(height=0.3)
            drone.hover(1)

            drone.goTo([0.0, -0.3, 1.3, 0])
            drone.hover(2)

            drone.trajectory_battery_check()

            print('Go home before landing...')
            drone.goTo([drone.pose_home[0], drone.pose_home[1], 0.3, 0])
            drone.hover(2)
            drone.land()
            print('Battery status: %.2f [V]' %drone.V_bat)
