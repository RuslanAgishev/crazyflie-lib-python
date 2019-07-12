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
from vispy import scene
from vispy.scene import visuals
from vispy.scene.cameras import TurntableCamera
import time
from threading import Thread
from multiprocessing import Process

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.crazyflie.high_level_commander import HighLevelCommander
from cflib.utils.multiranger import Multiranger

try:
    from sip import setapi

    setapi('QVariant', 2)
    setapi('QString', 2)
except ImportError:
    pass

from PyQt4 import QtGui, QtCore

logging.basicConfig(level=logging.INFO)
URI = 'radio://0/80/2M/E7E7E7E701'
if len(sys.argv) > 1:
    URI = sys.argv[1]

# Enable plotting of Crazyflie
PLOT_CF = False
# Enable plotting of down sensor
PLOT_SENSOR_UP = False
PLOT_SENSOR_DOWN = False
# Set the sensor threashold (in mm)
SENSOR_TH = 1500
# Set the speed factor for moving and rotating
SPEED_FACTOR = 1.0
# Battery voltage to land
BATTERY_LAND = 3.65

def is_close(range):
    MIN_DISTANCE = 300  # mm

    if range is None:
        return False
    else:
        return range < MIN_DISTANCE


class MainWindow(QtGui.QMainWindow):

    def __init__(self, URI):
        QtGui.QMainWindow.__init__(self)

        self.resize(1400, 1000)
        self.setWindowTitle('Multi-ranger point cloud')

        self.canvas = Canvas()
        self.canvas.create_native()
        self.canvas.native.setParent(self)

        self.setCentralWidget(self.canvas.native)

        cflib.crtp.init_drivers(enable_debug_driver=False)
        self.cf = Crazyflie(ro_cache=None, rw_cache='cache')

        # Connect callbacks from the Crazyflie API
        self.cf.connected.add_callback(self.connected)
        self.cf.disconnected.add_callback(self.disconnected)

        # Connect to the Crazyflie
        self.cf.open_link(URI)

        self.motion_commander = MotionCommander(self.cf)
        self.high_level_commander = HighLevelCommander(self.cf)
        self.multiranger = Multiranger(self.cf)
        time.sleep(2)

        self.motion_commander.take_off(0.2, 0.2)
        self.x_home = self.position[0]
        self.y_home = self.position[1]
        print('Home position:', self.x_home, self.y_home)
        time.sleep(1)
        self.motion_commander.start_forward(0.10 * SPEED_FACTOR)
        time.sleep(0.5)
        # self.hover = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0, 'height': 0.2}

        self.hoverTimer = QtCore.QTimer()
        self.hoverTimer.timeout.connect(self.sendHoverCommand)
        self.hoverTimer.setInterval(1000)
        self.hoverTimer.start()


    def sendHoverCommand(self):
        print(f"range.front = {self.measurement['front']}")
        print(f"range.left = {self.measurement['left']}")
        print(f"range.right = {self.measurement['right']}")
        if self.V_bat < BATTERY_LAND:
            # current_pose = self.position
            # current_yaw = get current yaw
            # self.motion_commander.turn_left(till current yaw)
            # self.motion_commander.move_distance(home_pose - current_pose)
            print('Landing...')
            self.motion_commander.land(0.1)
            time.sleep(0.5)
            
        if is_close(self.measurement['front']):
            self.motion_commander.turn_right(90, 180)
            time.sleep(0.1)
            self.motion_commander.start_forward(0.1 * SPEED_FACTOR)
        if is_close(self.measurement['front']) and self.measurement['left'] > self.measurement['right']:
            self.motion_commander.turn_left(90, 180)
            time.sleep(0.1)
            self.motion_commander.start_forward(0.1 * SPEED_FACTOR)
        if is_close(self.measurement['front']) and self.measurement['left'] < self.measurement['right']:
            self.motion_commander.turn_right(90, 180)
            time.sleep(0.1)
            self.motion_commander.start_forward(0.1 * SPEED_FACTOR)
        if is_close(self.measurement['left']):
            self.motion_commander.turn_right(45, 90)
            time.sleep(0.1)
            self.motion_commander.start_forward(0.1 * SPEED_FACTOR)
        if is_close(self.measurement['right']):
            self.motion_commander.turn_left(45, 90)
            time.sleep(0.1)
            self.motion_commander.start_forward(0.1 * SPEED_FACTOR)

    # def updateHover(self, k, v):
    #     if k != 'height':
    #         self.hover[k] = v * SPEED_FACTOR
    #     else:
    #         self.hover[k] += v

    def disconnected(self, URI):
        print('Disconnected')

    def connected(self, URI):
        print('We are now connected to {}'.format(URI))

        # The definition of the logconfig can be made before connecting
        lpos = LogConfig(name='Position', period_in_ms=100)
        lpos.add_variable('stateEstimate.x')
        lpos.add_variable('stateEstimate.y')
        lpos.add_variable('stateEstimate.z')
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
        lbat.add_variable('pm.vbat')
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
        position = [
            data['stateEstimate.x'],
            data['stateEstimate.y'],
            data['stateEstimate.z']
        ]
        self.position = position
        self.canvas.set_position(position)

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
        self.canvas.set_measurement(measurement)

    def battery_data(self, timestamp, data, logconf):
        self.V_bat = data['pm.vbat']
        print('Battery status: %.2f [V]' %self.V_bat)

    def closeEvent(self, event):
        if self.cf is not None:
            self.cf.close_link()


class Canvas(scene.SceneCanvas):
    def __init__(self):
        scene.SceneCanvas.__init__(self, keys=None)
        self.size = 1300, 900
        self.unfreeze()
        self.view = self.central_widget.add_view()
        self.view.bgcolor = '#ffffff'
        self.view.camera = TurntableCamera(
            fov=10.0, distance=30.0, up='+z', center=(0.0, 0.0, 0.0))
        self.last_pos = [0, 0, 0]
        self.pos_markers = visuals.Markers()
        self.meas_markers = visuals.Markers()
        self.pos_data = np.array([0, 0, 0], ndmin=2)
        self.meas_data = np.array([0, 0, 0], ndmin=2)
        self.lines = []

        self.view.add(self.pos_markers)
        self.view.add(self.meas_markers)
        for i in range(6):
            line = visuals.Line()
            self.lines.append(line)
            self.view.add(line)

        self.freeze()

        scene.visuals.XYZAxis(parent=self.view.scene)

    def set_position(self, pos):
        self.last_pos = pos
        if PLOT_CF:
            self.pos_data = np.append(self.pos_data, [pos], axis=0)
            self.pos_markers.set_data(self.pos_data, face_color='red', size=5)

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
                self.lines[i].set_data(np.array([o, data[i]]))
            else:
                self.lines[i].set_data(np.array([o, o]))

        if len(data) > 0:
            self.meas_data = np.append(self.meas_data, data, axis=0)
        self.meas_markers.set_data(self.meas_data, face_color='blue', size=5)


if __name__ == '__main__':
    appQt = QtGui.QApplication(sys.argv)
    win = MainWindow(URI)
    win.show()
    # th_1, th_2, th_3 = Thread(target=win.sendHoverCommand), Thread(target=appQt.exec_), Thread(target=win.canvas.set_measurement(win.measurement))
    # th_1.start(), th_2.start(), th_3.start()
    # th_1.join(), th_2.join(), th_3.join()
    appQt.exec_()
