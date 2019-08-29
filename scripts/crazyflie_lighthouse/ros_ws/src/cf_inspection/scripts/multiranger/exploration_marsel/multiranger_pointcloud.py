#!/usr/bin/env python
# -*- coding: utf-8 -*-

import logging
import math
import sys
import csv

import numpy as np
import time
from threading import Thread
import thread
from multiprocessing import Process

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils.multiranger import Multiranger

import rospy
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import PointCloud2, PointField


logging.basicConfig(level=logging.INFO)
URI = 'radio://0/80/2M/E7E7E7E701'
if len(sys.argv) > 1:
    URI = sys.argv[1]

# Enable plotting of Crazyflie
PLOT_CF = 0
# Set the sensor threashold (in mm)
SENSOR_TH = 1500
# Set the speed factor for moving and rotating
SPEED_FACTOR = 0.1

V_BATTERY_TO_GO_HOME = 3.4 # [V]
V_BATTERY_CHARGED = 3.9    # [V]

WRITE_TO_FILE = 0



def is_close(range):
    MIN_DISTANCE = 350  # mm

    if range is None:
        return False
    else:
        return range < MIN_DISTANCE


class Drone:
    def __init__(self, URI):

        cflib.crtp.init_drivers(enable_debug_driver=False)
        self.cf = Crazyflie(ro_cache=None, rw_cache='./cache')
        # Connect callbacks from the Crazyflie API
        self.cf.connected.add_callback(self.connected)
        self.cf.disconnected.add_callback(self.disconnected)
        # Connect to the Crazyflie
        self.cf.open_link(URI)
        # Tool to process the data from drone's sensors
        self.processing = Processing()
        
        self.motion_commander = MotionCommander(self.cf)
        time.sleep(3)
        self.motion_commander.take_off(0.3, 0.2)
        time.sleep(1)
        self.motion_commander.start_forward(0.15)
        time.sleep(0.5)

        self.stop_recording = []
        thread.start_new_thread(self.land_command, ())

        self.start_mission()

    def land_command(self):
        raw_input()
        self.stop_recording.append(True)
        print('Landing...')
        self.motion_commander.stop()
        self.motion_commander.land(0.2)
        time.sleep(1)

    def start_mission(self):
        rate = rospy.Rate(4000)
        while not self.stop_recording:
            print('fly')
            self.sendHoverCommand()
            rate.sleep()

    def sendHoverCommand(self):
        """
        Change UAV flight direction based on obstacles detected with multiranger.
        """
        if is_close(self.measurement['front']) and self.measurement['left'] > self.measurement['right']:
            print('FRONT RIGHT')
            self.motion_commander.stop()
            self.motion_commander.turn_left(60, 70)
            self.motion_commander.start_forward(0.15)
        if is_close(self.measurement['front']) and self.measurement['left'] < self.measurement['right']:
            print('FRONT LEFT')
            self.motion_commander.stop()
            self.motion_commander.turn_right(60, 70)
            self.motion_commander.start_forward(0.15)
        if is_close(self.measurement['left']):
            print('LEFT')
            self.motion_commander.right(0.1, 0.2)
            self.motion_commander.stop()
            self.motion_commander.turn_right(45, 70)
            self.motion_commander.start_forward(0.15)
        if is_close(self.measurement['right']):
            print('RIGHT')
            self.motion_commander.left(0.1, 0.2)
            self.motion_commander.stop()
            self.motion_commander.turn_left(45, 70)
            self.motion_commander.start_forward(0.15)


    def disconnected(self, URI):
        print('Disconnected')

    def connected(self, URI):
        print('We are now connected to {}'.format(URI))

        # The definition of the logconfig can be made before connecting
        lpos = LogConfig(name='Position', period_in_ms=100)
        lpos.add_variable('lighthouse.x')
        lpos.add_variable('lighthouse.y')
        lpos.add_variable('lighthouse.z')
        lpos.add_variable('stabilizer.roll')
        lpos.add_variable('stabilizer.pitch')
        lpos.add_variable('stabilizer.yaw')
        try:
            self.cf.log.add_config(lpos)
            lpos.data_received_cb.add_callback(self.pos_data)
            lpos.start()
        except KeyError as e:
            print('Could not start log configuration,'
                  '{} not found in TOC'.format(str(e)))
        except AttributeError:
            print('Could not add Position log config, bad configuration.')

        lmeas = LogConfig(name='Meas', period_in_ms=100)
        lmeas.add_variable('range.front')
        lmeas.add_variable('range.back')
        lmeas.add_variable('range.up')
        lmeas.add_variable('range.left')
        lmeas.add_variable('range.right')
        lmeas.add_variable('range.zrange')
        lmeas.add_variable('stabilizer.roll')
        lmeas.add_variable('stabilizer.pitch')
        lmeas.add_variable('stabilizer.yaw')
        try:
            self.cf.log.add_config(lmeas)
            lmeas.data_received_cb.add_callback(self.meas_data)
            lmeas.start()
        except KeyError as e:
            print('Could not start log configuration,'
                  '{} not found in TOC'.format(str(e)))
        except AttributeError:
            print('Could not add Measurement log config, bad configuration.')

        lbat = LogConfig(name='Battery', period_in_ms=500) # read battery status with 2 Hz rate
        lbat.add_variable('pm.vbat', 'float')
        try:
            self.cf.log.add_config(lbat)
            lbat.data_received_cb.add_callback(self.battery_data)
            lbat.start()
        except KeyError as e:
            print('Could not start log configuration,'
                  '{} not found in TOC'.format(str(e)))
        except AttributeError:
            print('Could not add Measurement log config, bad configuration.')

    def pos_data(self, timestamp, data, logconf):
        # Transformation according to https://wiki.bitcraze.io/doc:lighthouse:setup
        position = [
            -data['lighthouse.z'],
            -data['lighthouse.x'],
            data['lighthouse.y']

            # data['stateEstimate.x'],
            # data['stateEstimate.y'],
            # data['stateEstimate.z']
        ]
        orientation = [
            data['stabilizer.roll'],
            data['stabilizer.pitch'],
            data['stabilizer.yaw']
        ]
        self.position = position
        self.orientation = orientation
        self.processing.set_position(position, orientation)

    def meas_data(self, timestamp, data, logconf):
        measurement = {
            'roll': data['stabilizer.roll'],
            'pitch': data['stabilizer.pitch'],
            'yaw': data['stabilizer.yaw'],
            'front': data['range.front'],
            'back': data['range.back'],
            'up': data['range.up'],
            'down': data['range.zrange'],
            'left': data['range.left'],
            'right': data['range.right']
        }
        self.measurement = measurement
        self.processing.set_measurement(measurement)


    def battery_data(self, timestamp, data, logconf):
        self.V_bat = data['pm.vbat']
        # print('Battery status: %.2f [V]' %self.V_bat)
        if self.V_bat <= V_BATTERY_TO_GO_HOME:
            self.battery_state = 'needs_charging'
            # print('Battery is not charged: %.2f' %self.V_bat)
        elif self.V_bat >= V_BATTERY_CHARGED:
            self.battery_state = 'fully_charged'


class Processing:
    def __init__(self):
        self.last_pos = [0, 0, 0]
        self.path = Path()
        self.meas_data = np.array([0, 0, 0], ndmin=2)
        self.lines = []

    def msg_def_PoseStamped(self, pose, orient):
        worldFrame = "base_link"
        msg = PoseStamped()
        msg.header.seq = 0
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = worldFrame
        msg.pose.position.x = pose[0]
        msg.pose.position.y = pose[1]
        msg.pose.position.z = pose[2]
        orient = np.array(orient) / 180*3.14
        quaternion = quaternion_from_euler(orient[0], orient[1], orient[2]) #1.57
        msg.pose.orientation.x = quaternion[0]
        msg.pose.orientation.y = quaternion[1]
        msg.pose.orientation.z = quaternion[2]
        msg.pose.orientation.w = quaternion[3]
        msg.header.seq += 1
        return msg
    def publish_pose(self, pose, orient, topic_name):
        msg = self.msg_def_PoseStamped(pose, orient)
        pub = rospy.Publisher(topic_name, PoseStamped, queue_size=1)
        pub.publish(msg)
    def publish_path(self,path, pose, orient, topic_name, limit=-1):
        msg = self.msg_def_PoseStamped(pose, orient)
        path.header = msg.header
        path.poses.append(msg)
        if limit>0:
            path.poses = path.poses[-limit:]
        pub = rospy.Publisher(topic_name, Path, queue_size=1)
        pub.publish(path)

    def set_position(self, pose, orient):
        self.last_pos = pose
        if PLOT_CF:
            # publish to ROS topic for visualization:
            self.publish_pose(pose, orient, 'cf_pose')
            self.publish_path(self.path, pose, orient, 'cf_path', limit=200)

    def rot(self, roll, pitch, yaw, origin, point):
        cosr = math.cos(math.radians(roll))
        cosp = math.cos(math.radians(pitch))
        cosy = math.cos(math.radians(yaw))

        sinr = math.sin(math.radians(roll))
        sinp = math.sin(math.radians(pitch))
        siny = math.sin(math.radians(yaw))

        roty = np.array([[cosy, -siny, 0],
                         [siny, cosy, 0],
                         [0, 0, 1]])

        rotp = np.array([[cosp, 0, sinp],
                         [0, 1, 0],
                         [-sinp, 0, cosp]])

        rotr = np.array([[1, 0, 0],
                         [0, cosr, -sinr],
                         [0, sinr, cosr]])

        rotFirst = np.dot(rotr, rotp)

        rot = np.array(np.dot(rotFirst, roty))

        tmp = np.subtract(point, origin)
        tmp2 = np.dot(rot, tmp)
        return np.add(tmp2, origin)

    def rotate_and_create_points(self, m):
        data = []
        o = self.last_pos
        roll = m['roll']
        pitch = -m['pitch']
        yaw = m['yaw']

        if (m['left'] < SENSOR_TH):
            left = [o[0], o[1] + m['left'] / 1000.0, o[2]]
            data.append(self.rot(roll, pitch, yaw, o, left))

        if (m['right'] < SENSOR_TH):
            right = [o[0], o[1] - m['right'] / 1000.0, o[2]]
            data.append(self.rot(roll, pitch, yaw, o, right))

        if (m['front'] < SENSOR_TH):
            front = [o[0] + m['front'] / 1000.0, o[1], o[2]]
            data.append(self.rot(roll, pitch, yaw, o, front))

        if (m['back'] < SENSOR_TH):
            back = [o[0] - m['back'] / 1000.0, o[1], o[2]]
            data.append(self.rot(roll, pitch, yaw, o, back))

        return data

    def set_measurement(self, measurements=None):
        data = self.rotate_and_create_points(measurements)
        o = self.last_pos
        if len(data) > 0:
            self.meas_data = np.append(self.meas_data, data, axis=0)
        # ROS visualization of a PointCloud
        if PLOT_CF: self.publish_pointcloud(self.meas_data, 'multiranger_pointcloud', limit=2000)
        if WRITE_TO_FILE: self.write_to_file()

    def xyz_array_to_pointcloud2(self, points, limit):
        '''
        Create a sensor_msgs.PointCloud2 from an array of points.
        '''
        if limit>0: points = points[-limit:]
        msg = PointCloud2()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "base_link"
        if len(points.shape) == 3:
            msg.height = points.shape[1]
            msg.width = points.shape[0]
        else:
            msg.height = 1
            msg.width = len(points)
        msg.fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1)]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = 12*points.shape[0]
        msg.is_dense = int(np.isfinite(points).all())
        msg.data = np.asarray(points, np.float32).tostring()
        return msg 
    def publish_pointcloud(self, points, topic_name, limit=-1):
        msg = self.xyz_array_to_pointcloud2(points, limit)
        pub = rospy.Publisher(topic_name, PointCloud2, queue_size=1)
        pub.publish(msg)

    def write_to_file(self):
        with open('coordinates.csv', mode='w') as file:
            writer = csv.writer(file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
            writer.writerow(self.meas_data)


if __name__ == '__main__':
    rospy.init_node('drone_multiranger')
    drone = Drone(URI)
