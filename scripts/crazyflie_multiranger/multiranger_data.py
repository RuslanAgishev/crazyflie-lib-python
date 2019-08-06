"""
There's additional setting for (see constants below):
 * Plotting the downwards sensor
 * Plotting the estimated Crazyflie postition
 * Max threashold for sensors
 * Speed factor that set's how fast the Crazyflie moves
The demo is ended by either closing the graph window.
"""
import logging
import math
import sys

import numpy as np
import time
from threading import Thread
from multiprocessing import Process

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils.multiranger import Multiranger


logging.basicConfig(level=logging.INFO)
URI = 'radio://0/80/2M/E7E7E7E701'
if len(sys.argv) > 1:
    URI = sys.argv[1]

# Enable plotting of Crazyflie
PLOT_CF = False
# Enable plotting of down sensor
PLOT_SENSOR_UP = True
PLOT_SENSOR_DOWN = True
# Set the sensor threashold (in mm)
SENSOR_TH = 1500
# Set the speed factor for moving and rotating
SPEED_FACTOR = 0.1

V_BATTERY_TO_GO_HOME = 3.4
V_BATTERY_CHARGED = 3.9

def is_close(range):
    MIN_DISTANCE = 350  # mm

    if range is None:
        return False
    else:
        return range < MIN_DISTANCE


class Drone:
    def __init__(self, URI):
        cflib.crtp.init_drivers(enable_debug_driver=False)
        self.cf = Crazyflie(ro_cache=None, rw_cache='cache')
        # Connect callbacks from the Crazyflie API
        self.cf.connected.add_callback(self.connected)
        self.cf.disconnected.add_callback(self.disconnected)
        # Connect to the Crazyflie
        self.cf.open_link(URI)

        self.processing = Processing()

    def disconnected(self, URI):
        print('Disconnected')

    def connected(self, URI):
        print('We are now connected to {}'.format(URI))

        # The definition of the logconfig can be made before connecting
        lpos = LogConfig(name='Position', period_in_ms=100)
        lpos.add_variable('lighthouse.x')
        lpos.add_variable('lighthouse.y')
        lpos.add_variable('lighthouse.z')
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
        ]
        self.position = position
        self.processing.set_position(position)

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
        print('Battery status: %.2f [V]' %self.V_bat)
        if self.V_bat <= V_BATTERY_TO_GO_HOME:
            self.battery_state = 'needs_charging'
            print('Battery is not charged: %.2f' %self.V_bat)
        elif self.V_bat >= V_BATTERY_CHARGED:
            self.battery_state = 'fully_charged'


class Processing:
    def __init__(self):
        self.last_pos = [0, 0, 0]
        self.pos_data = np.array([0, 0, 0], ndmin=2)
        self.meas_data = np.array([0, 0, 0], ndmin=2)
        self.lines = []

    def set_position(self, pos):
        self.last_pos = pos
        if PLOT_CF:
            self.pos_data = np.append(self.pos_data, [pos], axis=0)

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

        if (m['up'] < SENSOR_TH and PLOT_SENSOR_UP):
            up = [o[0], o[1], o[2] + m['up'] / 1000.0]
            data.append(self.rot(roll, pitch, yaw, o, up))

        if (m['down'] < SENSOR_TH and PLOT_SENSOR_DOWN):
            down = [o[0], o[1], o[2] - m['down'] / 1000.0]
            data.append(self.rot(roll, pitch, yaw, o, down))

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
        for i in range(6):
            if i < len(data):
                o = self.last_pos
                # self.lines[i].set_data(np.array([o, data[i]]))
            # else:
                # self.lines[i].set_data(np.array([o, o]))

        if len(data) > 0:
            self.meas_data = np.append(self.meas_data, data, axis=0)
        # self.meas_markers.set_data(self.meas_data, face_color='blue', size=5)


if __name__ == '__main__':
    drone = Drone(URI)
