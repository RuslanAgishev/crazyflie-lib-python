
# coding: utf-8

# ## LIDAR to 2D grid map example
# 
# This simple tutorial shows how to read LIDAR (range) measurements from a file and convert it to occupancy grid.
# 
# Occupancy grid maps (_Hans Moravec, A.E. Elfes: High resolution maps from wide angle sonar, Proc. IEEE Int. Conf. Robotics Autom. (1985)_) are a popular, probabilistic approach to represent the environment. The grid is basically discrete representation of the environment, which shows if a grid cell is occupied or not. Here the map is represented as a `numpy array`, and numbers close to 1 means the cell is occupied (_marked with red on the next image_), numbers close to 0 means they are free (_marked with green_). The grid has the ability to represent unknown (unobserved) areas, which are close to 0.5.
# 
# ![Example](grid_map_example.png)
# 
# 
# In order to construct the grid map from the measurement we need to discretise the values. But, first let's need to `import` some necessary packages.


import math
import numpy as np
import matplotlib.pyplot as plt
from math import cos, sin, radians, pi
from collections import deque


# The measurement file contains the laser beam coordinates measurements
# in a `csv` (comma separated values) format.
def file_read(f):
    """
    Reading LIDAR laser beams (x,y and z global coordinates of the scans)
    """
    measures = [line.split(",") for line in open(f)]
    X = []
    Y = []
    Z = []
    for measure in measures:
        X.append(float(measure[0]))
        Y.append(float(measure[1]))
        Z.append(float(measure[2]))
    X = np.array(X)
    Y = np.array(Y)
    Z = np.array(Z)
    return X,Y,Z



# Handy functions which can used to convert a 2D range measurement to a grid map.
# For example the `bresenham`  gives the a straight line between two points in a grid map.
def bresenham(start, end):
    """
    Implementation of Bresenham's line drawing algorithm
    See en.wikipedia.org/wiki/Bresenham's_line_algorithm
    Bresenham's Line Algorithm
    Produces a np.array from start and end (original from roguebasin.com)
    >>> points1 = bresenham((4, 4), (6, 10))
    >>> print(points1)
    np.array([[4,4], [4,5], [5,6], [5,7], [5,8], [6,9], [6,10]])
    """
    # setup initial conditions
    x1, y1 = start
    x2, y2 = end
    dx = x2 - x1
    dy = y2 - y1
    is_steep = abs(dy) > abs(dx) # determine how steep the line is
    if is_steep: # rotate line
        x1, y1 = y1, x1
        x2, y2 = y2, x2
    swapped = False # swap start and end points if necessary and store swap state
    if x1 > x2:
        x1, x2 = x2, x1
        y1, y2 = y2, y1
        swapped = True
    dx = x2 - x1 # recalculate differentials
    dy = y2 - y1 # recalculate differentials
    error = int(dx / 2.0) # calculate error
    ystep = 1 if y1 < y2 else -1
    # iterate over bounding box generating points between start and end
    y = y1
    points = []
    for x in range(x1, x2 + 1):
        coord = [y, x] if is_steep else (x, y)
        points.append(coord)   
        error -= abs(dy)
        if error < 0:
            y += ystep
            error += dx
    if swapped: # reverse the list if the coordinates were swapped
        points.reverse()
    points = np.array(points)
    return points


EXTEND_AREA = 3.0
def calc_grid_map_config(ox, oy, xyreso):
    """
    Calculates the size, and the maximum distances according to the the measurement center
    """
    minx = round(min(ox) - EXTEND_AREA / 2.0)
    miny = round(min(oy) - EXTEND_AREA / 2.0)
    maxx = round(max(ox) + EXTEND_AREA / 2.0)
    maxy = round(max(oy) + EXTEND_AREA / 2.0)
    xw = int(round((maxx - minx) / xyreso))
    yw = int(round((maxy - miny) / xyreso))
    # print("The grid map is ", xw, "x", yw, ".")
    return minx, miny, maxx, maxy, xw, yw

def meters2grid(pose_m, nrows=500, ncols=500):
    # [0, 0](m) -> [250, 250]
    # [1, 0](m) -> [250+100, 250]
    # [0,-1](m) -> [250, 250-100]
    if np.isscalar(pose_m):
        pose_on_grid = int( pose_m*100 + ncols/2 )
    else:
        pose_on_grid = np.array( np.array(pose_m)*100 + np.array([ncols/2, nrows/2]), dtype=int )
    return pose_on_grid

def update_ray_casting_grid_map(pmap, scan_x, scan_y, robot_x, robot_y, params):
    """
    The breshen boolean tells if it's computed with bresenham ray casting (True) or with flood fill (False)
    """
    minx, miny, maxx, maxy = params['minx'], params['miny'], params['maxx'], params['maxy']
    xyreso = params['xyreso']

    robot_ix = int(round( (robot_x - minx) / xyreso))
    robot_iy = int(round( (robot_y - miny) / xyreso))

    # occupancy grid computed with bresenham ray casting
    for (x, y) in zip(scan_x, scan_y):
        ix = int(round((x - minx) / xyreso)) # x coordinate of the the occupied area
        iy = int(round((y - miny) / xyreso)) # y coordinate of the the occupied area

        laser_beams = bresenham((robot_ix, robot_iy), (ix, iy)) # line form the lidar to the cooupied point
        for laser_beam in laser_beams:
            pmap[laser_beam[0]][laser_beam[1]] = 0.0 # free area 0.0
        pmap[ix][iy] = 1.0     # occupied area 1.0
        pmap[ix+1][iy] = 1.0   # extend the occupied area
        pmap[ix][iy+1] = 1.0   # extend the occupied area
        pmap[ix+1][iy+1] = 1.0 # extend the occupied area

    return pmap

def plot_robot(robot_x, robot_y, robot_path, params):
    minx, miny, maxx, maxy = params['minx'], params['miny'], params['maxx'], params['maxy']
    xyreso = params['xyreso']
    robot_ix = int(round( (robot_x - minx) / xyreso))
    robot_iy = int(round( (robot_y - miny) / xyreso))
    plt.plot(robot_iy, robot_ix, 'ro', markersize=5)
    robot_path.append( [robot_ix, robot_iy] )
    path = np.array(robot_path)
    plt.plot(path[:,1], path[:,0], color='k')
    return robot_path

params = {
    'minx': -2.5,
    'miny': -2.5,
    'maxx': 2.5,
    'maxy': 2.5,
    'xyreso': 0.04,
}


if __name__ == '__main__':

    xyreso = 0.04  # x-y grid resolution
    yawreso = math.radians(3)  # yaw angle resolution [rad]
    X,Y,Z = file_read("coordsXYZ1565339797.95.csv")

    fig = plt.figure(figsize=(10,10))
    plt.grid(True, which="minor", color="w", linewidth = .6, alpha = 0.5)


    minx, miny, maxx, maxy = params['minx'], params['miny'], params['maxx'], params['maxy']
    xyreso = params['xyreso']
    wx = int(round( (maxx - minx) / xyreso)) # center x coordinate of the grid map
    wy = int(round( (maxy - miny) / xyreso)) # center y coordinate of the grid map

    pmap = np.ones((wx, wy))/2

    # TODO: initial position of the robot
    robot_x = -0.5; robot_y = 0.5
    dx = 0.03; dy = 0.03 # TODO: we don't need these variables with real coordinates
    robot_path = []

    for limit in range(300, 1200-10, 10):
        scan_x = X[limit-10:limit]
        scan_y = Y[limit-10:limit]

        # TODO: Robot movement, update this section further with coordinates of
        # the real drone.
        if limit < 300+(1200-300)/4:
            robot_x += dx
        elif limit < 300+2*(1200-300)/4:
            robot_y -= dy
        elif limit < 300+3*(1200-300)/4:
            robot_x -= dx
        else:
            robot_y += dy

        pmap = update_ray_casting_grid_map(pmap, scan_x, scan_y, robot_x, robot_y, params)
        xyres = np.array(pmap).shape
        
        plt.cla()
        plt.imshow(pmap, cmap = "PiYG_r")
        plt.clim(-0.4, 1.4)
        plt.gca().set_xticks(np.arange(-.5, xyres[1], 1), minor = True)
        plt.gca().set_yticks(np.arange(-.5, xyres[0], 1), minor = True)
        robot_path = plot_robot(robot_x, robot_y, robot_path, params)

        plt.pause(0.1)
    
    # close windows if Enter-button is pressed
    plt.pause(0.1)
    input('Hit Enter to close')
    plt.close('all')

