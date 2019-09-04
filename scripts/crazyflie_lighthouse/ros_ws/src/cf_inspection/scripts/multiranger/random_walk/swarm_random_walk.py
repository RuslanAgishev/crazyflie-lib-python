"""
Random walk algorithm implementation for a mobile robot
equipped with 4 ranger sensors (front, back, left and right)
for obstacles detection

author: Ruslan Agishev (agishev_ruslan@mail.ru)
reference: https://ieeexplore.ieee.org/abstract/document/6850799/s
"""

import matplotlib.pyplot as plt
from matplotlib import patches
import numpy as np

import cflib.crtp
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger

import rospy
from multiranger import DroneMultiranger
import time
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


def prepare(drone):
    activate_high_level_commander(drone.scf)
    reset_estimator(drone.scf)
    activate_mellinger_controller(drone.scf, False)


def plot_arrow(x, y, yaw, length=0.1, width=0.1):  # pragma: no cover
    plt.arrow(x, y, length * np.cos(yaw), length * np.sin(yaw),
              head_length=width, head_width=width)
    plt.plot(x, y)

def plot_robot(pose, yaw, params):
	r = int(params.sensor_range_m)
	plt.plot([pose[0]-r*np.cos(yaw), pose[0]+r*np.cos(yaw)],
			 [pose[1]-r*np.sin(yaw), pose[1]+r*np.sin(yaw)], '--', color='b')
	plt.plot([pose[0]-r*np.cos(yaw+np.pi/2), pose[0]+r*np.cos(yaw+np.pi/2)],
		     [pose[1]-r*np.sin(yaw+np.pi/2), pose[1]+r*np.sin(yaw+np.pi/2)], '--', color='b')
	plt.plot(pose[0], pose[1], 'ro', markersize=5)
	plot_arrow(pose[0], pose[1], yaw)

def borders_check(pose, params):
	gmap = params.gmap

	r = int(100*params.sensor_range_m)
	back = [pose[0]-r*np.cos(pose[2]), pose[1]-r*np.sin(pose[2])]
	front = [pose[0]+r*np.cos(pose[2]), pose[1]+r*np.sin(pose[2])]
	right = [pose[0]+r*np.cos(pose[2]+np.pi/2), pose[1]+r*np.sin(pose[2]+np.pi/2)]
	left = [pose[0]-r*np.cos(pose[2]+np.pi/2), pose[1]-r*np.sin(pose[2]+np.pi/2)]

	pi = np.array(pose[:2], dtype=int)
	backi = np.array(back, dtype=int)
	fronti = np.array(front, dtype=int)
	lefti = np.array(left, dtype=int)
	righti = np.array(right, dtype=int)

	border = {
		'front': 0,
		'back':  0,
		'right': 0,
		'left':  0,
	}

	for i in np.arange(min(pi[0], fronti[0]), max(pi[0], fronti[0])+1):
		for j in np.arange(min(pi[1], fronti[1]), max(pi[1], fronti[1])+1):
			if gmap[j,i]:
				# print('FRONT collision')
				border['front'] = 1

	for i in np.arange(min(pi[0], backi[0]), max(pi[0], backi[0])+1):
		for j in np.arange(min(pi[1], backi[1]), max(pi[1], backi[1])+1):
			if gmap[j,i]:
				# print('BACK collision')
				border['back'] = 1

	for i in np.arange(min(pi[0], lefti[0]), max(pi[0], lefti[0])+1):
		for j in np.arange(min(pi[1], lefti[1]), max(pi[1], lefti[1])+1):
			if gmap[j,i]:
				# print('LEFT collision')
				border['left'] = 1

	for i in np.arange(min(pi[0], righti[0]), max(pi[0], righti[0])+1):
		for j in np.arange(min(pi[1], righti[1]), max(pi[1], righti[1])+1):
			if gmap[j,i]:
				# print('RIGHT collision')
				border['right'] = 1
	return border

def meters2grid(pose_m, nrows=200, ncols=200):
    # [0, 0](m) -> [100, 100]
    # [1, 0](m) -> [100+100, 100]
    # [0,-1](m) -> [100, 100-100]
    if np.isscalar(pose_m):
        pose_on_grid = int( pose_m*100 + ncols/2 )
    else:
        pose_on_grid = np.array( np.array(pose_m)*100 + np.array([ncols/2, nrows/2]), dtype=int )
    return pose_on_grid
def grid2meters(pose_grid, nrows=200, ncols=200):
    # [100, 100] -> [0, 0](m)
    # [100+100, 100] -> [1, 0](m)
    # [100, 100-100] -> [0,-1](m)
    if np.isscalar(pose_grid):
        pose_meters = (pose_grid - ncols/2) / 100.0
    else:
        pose_meters = ( np.array(pose_grid) - np.array([ncols/2, nrows/2]) ) / 100.0
    return pose_meters

def visualize(traj, pose_m, yaw, params):
	ax = plt.gca()
	ax.set_xlim([-2.5, 2.5])
	ax.set_ylim([-2.5, 2.5])
	# Draw virtual borders
	rect = patches.Rectangle((-params.map_length_m/2.,-params.map_width_m/2.),params.map_length_m,params.map_width_m,linewidth=1,edgecolor='k',facecolor='none')
	ax.add_patch(rect)
	plt.plot(traj[:,0], traj[:,1], 'g')
	plot_robot(pose_m, yaw, params)


def takeoff(drone, height=0.2):
    # takeoff to z=0.3 m:
    print('Takeoff...')
    drone.sp = np.zeros(4); drone.sp[:3] = drone.position
    dz = 0.02
    for i in range(int(height/dz)):
        drone.sp[2] += dz
        fly(drone)
        time.sleep(0.1)
def fly(drone):
	sp = drone.sp
	drone.cf.commander.send_position_setpoint(sp[0], sp[1], sp[2], sp[3])
def land(drone):
    print('Landing...')
    while drone.sp[2]>-0.1:
        drone.sp[2] -= 0.02
        fly(drone)
        time.sleep(0.1)
    stop(drone)
    # Make sure that the last packet leaves before the link is closed
    # since the message queue is not flushed before closing
    time.sleep(0.1)
def stop(drone):
    drone.cf.commander.send_stop_setpoint()

def left_shift(pose, r):
	left = [pose[0]+r*np.cos(pose[2]+np.pi/2), pose[1]+r*np.sin(pose[2]+np.pi/2)]
	return left
def right_shift(pose, r):
	right = [pose[0]-r*np.cos(pose[2]+np.pi/2), pose[1]-r*np.sin(pose[2]+np.pi/2)]
	return right
def back_shift(pose, r):
	back = [pose[0]-r*np.cos(pose[2]), pose[1]-r*np.sin(pose[2])]
	return back

def is_close(range):
    MIN_DISTANCE = 350 # mm
    if range is None:
        return False
    else:
        return range < MIN_DISTANCE


def explorasion_mission(drone, params):
	drone.pose_home = drone.position
	print('Home positions:', drone.pose_home)
	#    x,    y,      yaw
	pose = [meters2grid(drone.pose_home[0]), meters2grid(drone.pose_home[1]), 0.0]
	traj = grid2meters(pose[:2])
	
	for _ in range(params.numiters):
		vel = 10*params.vel
		pose[0] += vel*np.cos(pose[2])
		pose[1] += vel*np.sin(pose[2])

		border = borders_check(pose, params)
		# print(border)

		if border['right'] or border['front']:
			print('Border in FRONT or RIGHT')
			pose[2] -= np.pi/2 * np.random.uniform(0.1, 0.6)
		elif border['left']:
			print('Border on the LEFT')
			pose[2] += np.pi/2 * np.random.uniform(0.1, 0.6)

		if is_close(drone.measurement['front']) and drone.measurement['left'] > drone.measurement['right']:
			print('FRONT RIGHT')
			pose[:2] = back_shift(pose, 5)
			pose[2] += np.pi/2 * np.random.uniform(0.1, 0.6)
		if is_close(drone.measurement['front']) and drone.measurement['left'] < drone.measurement['right']:
			print('FRONT LEFT')
			pose[:2] = back_shift(pose, 5)
			pose[2] += np.pi/2 * np.random.uniform(0.1, 0.6)
		if is_close(drone.measurement['left']):
			print('LEFT')
			pose[:2] = right_shift(pose, 5)
		if is_close(drone.measurement['right']):
			print('RIGHT')
			pose[:2] = left_shift(pose, 5)


		pose_m = grid2meters(pose[:2])
		traj = np.vstack([traj, pose_m])
		
		drone.sp = [pose_m[0], pose_m[1], params.flight_height, np.degrees(pose[2])%360]

		if params.toFly:
			fly(drone)

			time.sleep(0.1)

	# visualize(traj,pose_m, pose[2], params)
	if params.toFly: land(drone)
	


class Params:
	def __init__(self):
		self.map_width_m = 2.4
		self.map_length_m = 2.4
		self.sensor_range_m = 0.1
		self.wall_thickness_m = 2*self.sensor_range_m
		self.simulation_time = 10 # [sec]
		self.numiters = 400
		self.vel = 0.15 # [m/s]
		self.uris = [
					# 'radio://0/80/2M/E7E7E7E701',
					'radio://0/80/2M/E7E7E7E702',
					'radio://0/80/2M/E7E7E7E703',
					]
		self.flight_height = 0.2 # [m]
		self.toFly = 1
		self.create_borders_grid_map()

	def create_borders_grid_map(self):
		WIDTH = int(100 * (self.map_width_m))
		LENGTH = int(100 * (self.map_length_m))
		border = int(100 * self.wall_thickness_m)
		gmap = np.zeros([WIDTH, LENGTH])
		# walls
		gmap[:border, :] = 1
		gmap[-border:, :] = 1
		gmap[:, :border] = 1
		gmap[:, -border:] = 1
		self.gmap = gmap


def main():
	rospy.init_node('swarm_random_walk')
	params = Params()
	drones = []
	for URI in params.uris:
		drone = DroneMultiranger(URI)
		drones.append(drone)
	time.sleep(3)

	if params.toFly:
		th1 = Thread(target=prepare, args=(drones[0],) )
		th2 = Thread(target=prepare, args=(drones[1],) )
		th1.start(); th2.start();
		th1.join(); th2.join();

		raw_input('Press Enter to fly...')
		th1 = Thread(target=takeoff, args=(drones[0], params.flight_height,) )
		th2 = Thread(target=takeoff, args=(drones[1], params.flight_height,) )
		th1.start(); th2.start();
		th1.join(); th2.join();
	
		# th1 = Thread(target=land, args=(drones[0],) )
		# th2 = Thread(target=land, args=(drones[1],) )
		# th1.start(); th2.start();
		# th1.join(); th2.join();

	time.sleep(0.2)

	th1 = Thread(target=explorasion_mission, args=(drones[0], params,) )
	th2 = Thread(target=explorasion_mission, args=(drones[1], params,) )
	th1.start(); th2.start();
	th1.join(); th2.join();


if __name__ == '__main__':
	try:
		main()
	except KeyboardInterrupt:
		pass
		