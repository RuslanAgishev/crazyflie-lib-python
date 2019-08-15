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

import cflib.crtp
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger

import rospy
from multiranger_scf import DroneMultiranger
from threading import Thread


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

            print("{} {} {}".
                  format(max_x - min_x, max_y - min_y, max_z - min_z))

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


def shift(arr, num, fill_value):
    result = np.empty_like(arr)
    if num > 0:
        result[:num] = fill_value
        result[num:] = arr[:-num]
    elif num < 0:
        result[num:] = fill_value
        result[:num] = arr[-num:]
    else:
        result[:] = arr
    return result

def prepare(scf):
    activate_high_level_commander(scf)
    reset_estimator(scf)
    activate_mellinger_controller(scf, False)


def square_flight(drone, numiters=8):
    commander = drone.scf.cf.high_level_commander

    commander.takeoff(0.15, 2.0)
    time.sleep(3)

    flight_time = 3
    dz = 0.0
    for _ in range(numiters):
        # one flight mission
        for goal in drone.waypoints:
            goal = np.array(goal)
            goal[2]+=dz; goal[3] *= (3.14/180)
            print('Going to', goal)
            commander.go_to(goal[0], goal[1], goal[2], goal[3], flight_time, relative=False)
            time.sleep(flight_time)
        dz += 0.05

    # commander.go_to(drone.pose_home[0], drone.pose_home[1], 0.1, 0, flight_time, relative=False)
    # time.sleep(flight_time)

    commander.land(0.0, 3)
    time.sleep(1)
    commander.stop()

def circle_trajectory(drone, angle_shift=0):
    commander = drone.cf.commander

    angular_range = np.linspace(0+angle_shift, 2*np.pi+angle_shift, 80)
    R = 0.7; h = 0.2; dh = 0.4
    numiters = 4
    t0 = time.time()
    for _ in range(numiters):
        print('Height: %.2f [m]' %h)
        # circular trajectory
        for t in angular_range:
            sp = [R*np.cos(t), R*np.sin(t), h, 0]
            commander.send_position_setpoint(sp[0], sp[1], sp[2], sp[3])
            time.sleep(0.1)
            print('Time passed: %2.f [sec]' %(time.time()-t0))
        h += dh

    for _ in range(numiters):
        h -= dh
        print('Height: %.2f [m]' %h)
        # circular trajectory
        for t in angular_range:
            sp = [R*np.cos(t), R*np.sin(t), h, 0]
            commander.send_position_setpoint(sp[0], sp[1], sp[2], sp[3])
            time.sleep(0.1)
            print('Time passed: %2.f [sec]' %(time.time()-t0))


# Waypoints for square trajectories:
length = 1.6
square_waypoints = [
        [-length/2, length/2, 0.15, 0],
        [length/2,  length/2, 0.15, 0],
        [ length/2, -length/2, 0.15, 0],
        [-length/2, -length/2, 0.15, 0],
]
phi = np.pi/4
cos = np.cos(phi)
sin = np.sin(phi)
R = np.array([[cos, -sin, 0, 0],
              [sin,  cos, 0, 0],
              [0,    0,   1, 0],
              [0,    0,   0, 1]])
romb_waypoints = []
for wp in square_waypoints:
    wp_rot = np.dot(R, wp)
    romb_waypoints.append(wp_rot)
romb_waypoints = shift(romb_waypoints, 1, fill_value=romb_waypoints[-1])



# URI to the Crazyflie to connect to
URI1 = 'radio://0/80/2M/E7E7E7E703'
URI2 = 'radio://0/80/2M/E7E7E7E702'


if __name__ == '__main__':
    rospy.init_node('drone_multiranger')

    drone1 = DroneMultiranger(URI1)
    drone2 = DroneMultiranger(URI2)
    time.sleep(3)
    
    drone1.pose_home = drone1.position
    drone2.pose_home = drone2.position

    print('Home positions:', drone1.pose_home, drone2.pose_home)

    th1 = Thread(target=prepare, args=(drone1.scf,) )
    th2 = Thread(target=prepare, args=(drone2.scf,) )
    th1.start()
    th2.start()
    th1.join()
    th2.join()

    # drone1.waypoints = square_waypoints
    # drone2.waypoints = romb_waypoints
    # th1 = Thread(target=square_flight, args=(drone1,) )
    # th2 = Thread(target=square_flight, args=(drone2,) )

    th1 = Thread(target=circle_trajectory, args=(drone1,) )
    th2 = Thread(target=circle_trajectory, args=(drone2, np.pi,) )
    th1.start()
    th2.start()
    th1.join()
    th2.join()