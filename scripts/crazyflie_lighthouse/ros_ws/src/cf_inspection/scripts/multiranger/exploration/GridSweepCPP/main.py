import math
import matplotlib.pyplot as plt
import numpy as np
from grid_based_sweep_coverage_path_planner import planning
from connect_crazyflie import reset_estimator
from connect_crazyflie import activate_high_level_commander
from connect_crazyflie import activate_mellinger_controller

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
import time

import rospy
from drone_multiranger import DroneMultiranger
from grid_map import GridMap

    
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
def land(drone, height=-0.1, land_speed=0.2):
    print('Landing...')
    while drone.sp[2]>height:
        drone.sp[2] -= land_speed*0.1
        fly(drone)
        time.sleep(0.1)
    drone.cf.commander.send_stop_setpoint()
    time.sleep(0.1)
def goTo(drone, goal, pos_tol=0.03, yaw_tol=3):
    def normalize(vector):
        vector = np.array(vector)
        v_norm = vector / np.linalg.norm(vector) if np.linalg.norm(vector)!=0 else np.zeros_like(vector)
        return v_norm
    goal = np.array(goal)
    print('Going to', goal)
    if drone.sp is None:
        drone.sp = np.zeros(4); drone.sp[:3] = drone.pose
    while np.linalg.norm(goal[:3] - drone.sp[:3]) > pos_tol or np.linalg.norm(drone.sp[3]-goal[3]) > yaw_tol:
        n = normalize(goal[:3] - drone.sp[:3])
        drone.sp[:3] += 0.03 * n # position setpoints
        drone.sp[3] += 3 * np.sign( goal[3] - drone.sp[3] ) # yaw angle
        # print('Yaw', drone.sp[3], 'yaw diff', np.linalg.norm(drone.sp[3]-goal[3]))
        fly(drone)
        time.sleep(0.1)
def hover(drone, t_hover=2):
    t0 = time.time()
    print('Holding position for %.2f [sec]...' %t_hover)
    while time.time() - t0 < t_hover:
        fly(drone)
        time.sleep(0.1)

def left_shift(pose, r):
    left = pose
    left[:2] = [pose[0]+r*np.cos(pose[2]+np.pi/2), pose[1]+r*np.sin(pose[2]+np.pi/2)]
    return left
def right_shift(pose, r):
    right = pose
    right[:2] = [pose[0]-r*np.cos(pose[2]+np.pi/2), pose[1]-r*np.sin(pose[2]+np.pi/2)]
    return right
def back_shift(pose, r):
    back = pose
    back[:2] = [pose[0]-r*np.cos(pose[2]), pose[1]-r*np.sin(pose[2])]
    return back
def turn_left(pose, yaw=np.pi/2*np.random.uniform(0.2, 0.6)):
    pose[2] -= yaw
    return pose
def turn_right(pose, yaw=np.pi/2*np.random.uniform(0.2, 0.6)):
    pose[2] += yaw
    return pose
def slow_down(state, params, dv=0.1):
    if state[3]>params.min_vel:
        state[3] -= dv
    return state


def prepare(drone):
    scf = drone.scf
    activate_high_level_commander(scf)
    reset_estimator(scf)
    activate_mellinger_controller(scf, False)

def visualize(traj, pose, params):
    def plot_robot(pose, params):
        r = params.sensor_range_m
        plt.plot([pose[0]-r*np.cos(pose[2]), pose[0]+r*np.cos(pose[2])],
                 [pose[1]-r*np.sin(pose[2]), pose[1]+r*np.sin(pose[2])], '--', color='b')
        plt.plot([pose[0]-r*np.cos(pose[2]+np.pi/2), pose[0]+r*np.cos(pose[2]+np.pi/2)],
                 [pose[1]-r*np.sin(pose[2]+np.pi/2), pose[1]+r*np.sin(pose[2]+np.pi/2)], '--', color='b')
        plt.plot(pose[0], pose[1], 'ro', markersize=5)
        plt.arrow(pose[0], pose[1], 0.05 * np.cos(pose[2]), 0.05 * np.sin(pose[2]),
                  head_length=0.1, head_width=0.1)

    plt.plot(traj[:,0], traj[:,1], 'g')
    plot_robot(pose, params)
    plt.legend()

def obstacle_check(pose, gridmap, params):
    gmap = gridmap

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

    obstacle = {
        'front': 0,
        'back':  0,
        'right': 0,
        'left':  0,
    }

    for i in np.arange(min(pi[0], fronti[0]), max(pi[0], fronti[0])+1):
        for j in np.arange(min(pi[1], fronti[1]), max(pi[1], fronti[1])+1):
            m = min(j, gmap.shape[0]-1); n = min(i, gmap.shape[1]-1)
            if gmap[m,n]:
                # print('FRONT collision')
                obstacle['front'] = 1

    for i in np.arange(min(pi[0], backi[0]), max(pi[0], backi[0])+1):
        for j in np.arange(min(pi[1], backi[1]), max(pi[1], backi[1])+1):
            m = min(j, gmap.shape[0]-1); n = min(i, gmap.shape[1]-1)
            if gmap[m,n]:
                # print('BACK collision')
                obstacle['back'] = 1

    for i in np.arange(min(pi[0], lefti[0]), max(pi[0], lefti[0])+1):
        for j in np.arange(min(pi[1], lefti[1]), max(pi[1], lefti[1])+1):
            m = min(j, gmap.shape[0]-1); n = min(i, gmap.shape[1]-1)
            if gmap[m,n]:
                # print('LEFT collision')
                obstacle['left'] = 1

    for i in np.arange(min(pi[0], righti[0]), max(pi[0], righti[0])+1):
        for j in np.arange(min(pi[1], righti[1]), max(pi[1], righti[1])+1):
            m = min(j, gmap.shape[0]-1); n = min(i, gmap.shape[1]-1)
            if gmap[m,n]:
                # print('RIGHT collision')
                obstacle['right'] = 1

    return obstacle

def motion(state, goal, params):
    # state = [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
    dx = goal[0] - state[0]
    dy = goal[1] - state[1]
    goal_yaw = math.atan2(dy, dx)
    state[4] = 2.2*math.sin(goal_yaw - state[2]) # omega(rad/s)
    state[2] += params.dt*state[4] # yaw(rad)

    dist_to_goal = np.linalg.norm(goal - state[:2])
    state[3] += 0.1*dist_to_goal
    if state[3] >= params.max_vel: state[3] = params.max_vel
    if state[3] <= params.min_vel: state[3] = params.min_vel

    dv = params.dt*state[3]
    state[0] += dv*np.cos(state[2]) # x(m)
    state[1] += dv*np.sin(state[2]) # y(m)

    return state


def avoid_obstacles(drone, params):
    def is_close(measured_range):
        # MIN_DISTANCE = 1000*params.sensor_range_m # mm
        MIN_DISTANCE = 350 # mm
        if measured_range is None:
            return False
        else:
            return measured_range < MIN_DISTANCE
    pose = drone.state
    if is_close(drone.measurement['front']) and drone.measurement['left'] > drone.measurement['right']:
        print('FRONT RIGHT')
        pose = slow_down(pose, params)
        pose = back_shift(pose, 0.02)
        pose = turn_left(pose, np.radians(40))
    if is_close(drone.measurement['front']) and drone.measurement['left'] < drone.measurement['right']:
        print('FRONT LEFT')
        pose = slow_down(pose, params)
        pose = back_shift(pose, 0.02)
        pose = turn_right(pose, np.radians(40))
    if is_close(drone.measurement['left']):
        print('LEFT')
        pose = right_shift(pose, 0.08)
    if is_close(drone.measurement['right']):
        print('RIGHT')
        pose = left_shift(pose, 0.08)
    drone.state = np.array(pose)

def flight_mission(drone, goal_x, goal_y, params):
    goal = [goal_x[drone.goali], goal_y[drone.goali]] # goal = [x, y], m
    # initial state = [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
    drone.state = np.array([drone.position[0], drone.position[1], 0.0, 0.0, 0.0])
    drone.traj = drone.state[:2]
    t_prev_goal = time.time()

    plt.figure(figsize=(10,10))
    gridmap.draw_map()
    # while True:
    for _ in range(params.numiters):
        drone.state = motion(drone.state, goal, params)

        pose_grid = gridmap.meters2grid(drone.state[:2])
        boundary = obstacle_check([pose_grid[0], pose_grid[1], drone.state[2]], gridmap.gmap, params)
        # print(boundary)

        if boundary['right'] or boundary['front']:
            drone.state = slow_down(drone.state, params)
            drone.state = turn_left(drone.state, np.radians(20))
        elif boundary['left']:
            drone.state = slow_down(drone.state, params)
            drone.state = turn_right(drone.state, np.radians(20))

        if params.toFly: avoid_obstacles(drone, params)

        goal_dist = np.linalg.norm(goal - drone.state[:2])
        # print('Distance to goal %.2f [m]:' %goal_dist)
        t_current = time.time()
        if goal_dist < params.goal_tol or (t_current - t_prev_goal) > params.time_to_switch_goal: # goal is reached
            print('Switching to the next goal.')
            # print('Time from the previous reached goal:', t_current - t_prev_goal)
            if drone.goali < len(goal_x) - 1:
                drone.goali += 1
            else:
                drone.goali = 0
                break
            t_prev_goal = time.time()
            goal = np.array([goal_x[drone.goali], goal_y[drone.goali]])


        drone.sp = [drone.state[0], drone.state[1], params.flight_height, np.degrees(drone.state[2])%360]
        if params.toFly: fly(drone)
        drone.processing.publish_pose(drone.sp[:3], [0,0,0], 'drone_sp')

        drone.traj = np.vstack([drone.traj, drone.state[:2]])

        if params.check_battery:
            try:
                if drone.battery_state == 'needs_charging':
                    print('Going home to CHARGE the battery')
                    print('Battery status: %.2f [V]' %drone.V_bat)
                    break
            except:
                pass

        if params.animate or params.toFly:
            plt.cla()
            gridmap.draw_map()
            plt.plot(goal_x, goal_y)
            plt.plot(goal[0], goal[1], 'ro', markersize=20, label='Goal position')
            visualize(drone.traj, drone.state, params)
            plt.pause(0.1)

    if params.toFly:
        goTo(drone, [drone.pose_home[0], drone.pose_home[1], params.flight_height, 0])
        hover(drone, 1.0)
        land(drone)


def exploration_conveyer(drone, goal_x, goal_y, params):
    def land_to_charge(drone, params):
        for _ in range(params.land_to_charge_attempts):
            if drone.charging_state != 1:
                takeoff(drone, drone.pose_home[2]+0.2)
                goTo(drone, [drone.pose_home[0], drone.pose_home[1], drone.pose_home[2]+0.15, 0.0])
                hover(drone, t_hover=4)
                land(drone, height=drone.pose_home[2]+0.04, land_speed=0.05)
                time.sleep(4)
            else:
                print('Charging started')
                break
    # while True:
    for _ in range(params.num_missions):
        if params.check_battery:
            # Waiting for the battery to become charged
            while True:
                try:
                    if (drone.battery_state == 'fully_charged'):
                        print('Battery status: %.2f [V]' %drone.V_bat)
                        break
                except:
                    pass
        # One flight mission
        print("Starting the mission!")
        flight_mission(drone, goal_x, goal_y, params)
        time.sleep(4)
        if params.toFly: land_to_charge(drone, params)
        time.sleep(params.time_between_missions)


class Params:
    def __init__(self):
        self.numiters = 500
        self.vel = 0.5 # [m/s]
        self.uri = 'radio://0/80/2M/E7E7E7E702'
        self.flight_height = 0.4 # [m]
        self.toFly = 1
        self.check_battery = 1
        self.animate = 0
        self.dt = 0.1
        self.goal_tol = 0.3
        self.max_vel = 0.5 # m/s
        self.min_vel = 0.1 # m/s
        self.sensor_range_m = 0.3 # m
        self.time_to_switch_goal = 10.0 # sec
        self.land_to_charge_attempts = 3
        self.num_missions = 3
        self.time_between_missions = 5


if __name__ == '__main__':
    rospy.init_node('random_walk')
    params = Params()
    drone = DroneMultiranger(params.uri)
    time.sleep(3)
    drone.goali = 0
    drone.pose_home = drone.position
    print('Home positions:', drone.pose_home)

    SCALE = 1.3
    flight_area_vertices = SCALE * np.array([[-0.6, 0.8], [-0.9, -0.9], [0.8, -0.8], [0.5, 0.9]])
    gridmap = GridMap(flight_area_vertices)

    ox = flight_area_vertices[:,0].tolist() + [flight_area_vertices[0,0]]
    oy = flight_area_vertices[:,1].tolist() + [flight_area_vertices[0,1]]
    reso = params.goal_tol
    goal_x, goal_y = planning(ox, oy, reso)


    if params.toFly:
        prepare(drone)
        raw_input('Press Enter to fly...')
        takeoff(drone, params.flight_height)

    # flight_mission(drone, goal_x, goal_y, params)
    exploration_conveyer(drone, goal_x, goal_y, params)

    gridmap.draw_map()
    plt.plot(goal_x, goal_y)
    visualize(drone.traj, drone.state, params)

    raw_input('Hit Enter to close all figures.')
    plt.close('all')

    print('Mission is complete!')
    drone.disconnect()
    

