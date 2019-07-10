# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2017 Bitcraze AB
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
A script to fly 5 Crazyflies in formation. One stays in the center and the
other four fly aound it in a circle. Mainly intended to be used with the
Flow deck.
The starting positions are vital and should be oriented like this

     >

^    +    v

     <

The distance from the center to the perimeter of the circle is around 0.5 m

"""
import math
import time

import cflib.crtp
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm

# Change uris according to your setup
URI1 = 'radio://0/80/2M/E7E7E7E701'
URI2 = 'radio://0/80/2M/E7E7E7E702'
uris = {
         URI1,
         URI2,
       }
       
h1 = 0.4
h2 = 0.6
params = {
    URI1: [h1],
    URI2: [h2],
}

def reset_estimator(scf):
    cf = scf.cf
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')
    time.sleep(2)



def run_sequence(scf, params):
    cf = scf.cf

    Z = params
    for y in range(int(Z*25)):
            cf.commander.send_hover_setpoint(0, 0, 0, y / 25)
            time.sleep(0.1)

    for _ in range(20):
        cf.commander.send_hover_setpoint(0, 0, 0, Z)
        time.sleep(0.1)

    for _ in range(50):
        cf.commander.send_hover_setpoint(0.5, 0, 36 * 2, Z)
        time.sleep(0.1)

    for _ in range(50):
        cf.commander.send_hover_setpoint(0.5, 0, -36 * 2, Z)
        time.sleep(0.1)

    for _ in range(20):
        cf.commander.send_hover_setpoint(0, 0, 0, Z)
        time.sleep(0.1)

    for y in range(int(Z*25)):
        cf.commander.send_hover_setpoint(0, 0, 0, (int(Z*25) - y) / 25)
        time.sleep(0.1)

    cf.commander.send_stop_setpoint()


if __name__ == '__main__':
    cflib.crtp.init_drivers(enable_debug_driver=False)

    factory = CachedCfFactory(rw_cache='./cache')
    with Swarm(uris, factory=factory) as swarm:
        swarm.parallel_safe(reset_estimator)
        swarm.parallel_safe(run_sequence, args_dict=params)

        time.sleep(1)
