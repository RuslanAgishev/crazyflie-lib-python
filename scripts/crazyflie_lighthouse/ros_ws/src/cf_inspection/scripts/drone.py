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
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger

# ROS
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Float64


V_BATTERY_TO_GO_HOME = 3.4
V_BATTERY_CHARGED = 3.8

class Drone:
    def __init__(self, uri='radio://0/80/2M/E7E7E7E7E7'):
        self.uri = uri # URI to the Crazyflie to connect to
        self.id = self.uri[-2:]
        self.cf = SyncCrazyflie(self.uri, cf=Crazyflie(rw_cache='./cache')).cf
        self.pose = None
        self.pose_home = np.array([0,0,0])
        self.orient = None
        self.sp = None
        self.path = Path()
        self.battery_state = ''

    def fly(self):
        self.cf.commander.send_position_setpoint(self.sp[0], self.sp[1], self.sp[2], self.sp[3])

    def takeoff(self, height=0.3, toFly=True):
        # takeoff to z=0.3 m:
        print('Takeoff...')
        self.sp = np.zeros(4); self.sp[:3] = self.pose
        dz = 0.02
        for i in range(int(height/dz)):
            self.sp[2] += dz
            if toFly: self.fly()
            self.publish_sp()
            self.publish_path() if toFly else self.publish_path_sp()
            time.sleep(0.1)

    def land(self, toFly=True):
        print('Landing...')
        while self.sp[2]>-0.1:
            self.sp[2] -= 0.02
            if toFly: self.fly()
            self.publish_sp()
            self.publish_path() if toFly else self.publish_path_sp()
            time.sleep(0.1)
        self.stop()
        # Make sure that the last packet leaves before the link is closed
        # since the message queue is not flushed before closing
        time.sleep(0.1)
    def stop(self):
        self.cf.commander.send_stop_setpoint()

    def goTo(self, goal, pos_tol=0.03, yaw_tol=3, toFly=True):
        goal = np.array(goal)
        print('Going to', goal)
        if self.sp is None:
            self.sp = np.zeros(4); self.sp[:3] = self.pose
        while norm(goal[:3] - self.sp[:3]) > pos_tol or norm(self.sp[3]-goal[3]) > yaw_tol:
            n = normalize(goal[:3] - self.sp[:3])
            self.sp[:3] += 0.03 * n # position setpoints
            self.sp[3] += 3 * np.sign( goal[3] - self.sp[3] ) # yaw angle
            # print('Yaw', self.sp[3], 'yaw diff', norm(self.sp[3]-goal[3]))
            if toFly: self.fly()
            self.publish_sp()
            self.publish_path() if toFly else self.publish_path_sp()
            time.sleep(0.1)

    def hover(self, t_hover=2, toFly=True):
        t0 = time.time()
        while time.time() - t0 < t_hover:
            if toFly: self.fly()
            self.publish_sp()
            self.publish_path() if toFly else self.publish_path_sp()
            time.sleep(0.1)

    def trajectory(self, toFly=True):
        """ Figure 8 trajectory """
        # 1-st circle
        for _ in range(50):
            if toFly: self.cf.commander.send_hover_setpoint(0.5, 0, 36 * 2, 1.3)
            self.publish_path()
            time.sleep(0.1)
        # 2-nd circle
        for _ in range(50):
            if toFly: self.cf.commander.send_hover_setpoint(0.5, 0, -36 * 2, 1.3)
            self.publish_path()
            time.sleep(0.1)
        # hover for 2 sec
        for _ in range(20):
            if toFly: self.cf.commander.send_hover_setpoint(0, 0, 0, 1.3)
            self.publish_path()
            time.sleep(0.1)
    def trajectory_battery_check(self, toFly=True):
        """ Figure 8 trajectory """
        # 1-st circle
        for _ in range(50):
            if not self.battery_state == 'needs_charging':
                if toFly: self.cf.commander.send_hover_setpoint(0.5, 0, 36 * 2, 1.3)
                self.publish_path()
                time.sleep(0.1)
        # 2-nd circle
        for _ in range(50):
            if not self.battery_state == 'needs_charging':
                if toFly: self.cf.commander.send_hover_setpoint(0.5, 0, -36 * 2, 1.3)
                self.publish_path()
                time.sleep(0.1)
        # hover for 2 sec
        for _ in range(20):
            if not self.battery_state == 'needs_charging':
                if toFly: self.cf.commander.send_hover_setpoint(0, 0, 0, 1.3)
                self.publish_path()
                time.sleep(0.1)
    def publish_path_sp(self, limit=200):
        publish_path(self.path, self.sp[:3], self.orient, 'cf'+self.id+'_path', limit)
    def publish_path(self, limit=200):
        publish_path(self.path, self.pose, self.orient, 'cf'+self.id+'_path', limit)
    def publish_sp(self):
        publish_pose(self.sp[:3], np.array([0,0,self.sp[3]]), 'cf'+self.id+'_sp')    
    def position_callback(self, timestamp, data, logconf):
        x = data['kalman.stateX']
        y = data['kalman.stateY']
        z = data['kalman.stateZ']
        self.pose = np.array([x, y, z])
        roll = np.radians( data['stabilizer.roll'] )
        pitch = np.radians( data['stabilizer.pitch'] )
        yaw = np.radians( data['stabilizer.yaw'] )
        self.orient = np.array([roll, pitch, yaw])
        # publish to ROS topic for visualization:
        publish_pose(self.pose, self.orient, 'cf'+self.id+'_pose')
    def start_position_reading(self):
        log_conf = LogConfig(name='Position', period_in_ms=50) # read position with 20 Hz rate
        log_conf.add_variable('kalman.stateX', 'float')
        log_conf.add_variable('kalman.stateY', 'float')
        log_conf.add_variable('kalman.stateZ', 'float')
        log_conf.add_variable('stabilizer.roll', 'float')
        log_conf.add_variable('stabilizer.pitch', 'float')
        log_conf.add_variable('stabilizer.yaw', 'float')
        self.cf.log.add_config(log_conf)
        log_conf.data_received_cb.add_callback(self.position_callback)
        log_conf.start()

    def battery_callback(self, timestamp, data, logconf):
        self.V_bat = data['pm.vbat']
        # publish battery status as ROS msg
        pub = rospy.Publisher('cf'+self.id+'_Vbattery', Float64, queue_size=1)
        pub.publish(self.V_bat)
        # print('Battery status: %.2f [V]' %self.V_bat)
        if self.V_bat <= V_BATTERY_TO_GO_HOME:
            self.battery_state = 'needs_charging'
            print('Battery is not charged: %.2f' %self.V_bat)
        elif self.V_bat >= V_BATTERY_CHARGED:
            self.battery_state = 'fully_charged'
    def start_battery_status_reading(self):
        log_conf = LogConfig(name='Battery', period_in_ms=500) # read battery status with 2 Hz rate
        log_conf.add_variable('pm.vbat', 'float')
        self.cf.log.add_config(log_conf)
        log_conf.data_received_cb.add_callback(self.battery_callback)
        log_conf.start()


""" Helper functions """
def normalize(vector):
    vector = np.array(vector)
    v_norm = vector / norm(vector) if norm(vector)!=0 else np.zeros_like(vector)
    return v_norm
def euler_to_quaternion(roll, pitch, yaw):
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return [qx, qy, qz, qw]
def msg_def_PoseStamped(pose, orient):
    worldFrame = "base_link"
    msg = PoseStamped()
    msg.header.seq = 0
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = worldFrame
    msg.pose.position.x = pose[0]
    msg.pose.position.y = pose[1]
    msg.pose.position.z = pose[2]
    quaternion = euler_to_quaternion(orient[0], orient[1], orient[2]) #1.57
    msg.pose.orientation.x = quaternion[0]
    msg.pose.orientation.y = quaternion[1]
    msg.pose.orientation.z = quaternion[2]
    msg.pose.orientation.w = quaternion[3]
    msg.header.seq += 1
    return msg

def publish_pose(pose, orient, topic_name):
    msg = msg_def_PoseStamped(pose, orient)
    pub = rospy.Publisher(topic_name, PoseStamped, queue_size=1)
    pub.publish(msg)
def publish_path(path, pose, orient, topic_name, limit=-1):
    msg = msg_def_PoseStamped(pose, orient)
    path.header = msg.header
    path.poses.append(msg)
    if limit>0:
        path.poses = path.poses[-limit:]
    pub = rospy.Publisher(topic_name, Path, queue_size=1)
    pub.publish(path)