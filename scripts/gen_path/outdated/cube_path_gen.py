#!/usr/bin/env python
from dis import dis
from tracemalloc import start
import numpy as np
import csv

CCW = True
start_point = [0.0, 0.0]
distance = 4.0
height = [3.0, 6.0]
resolution = 50 * 9
turn_ratio = 0.2

p = 'path/outdoor/path17/'
f = open(p + 'plot_uav1.csv', 'w', newline='')
wr = csv.writer(f)
wr.writerow(['x', 'y', 'z', 'yaw'])

print("control\t0")
print("yaw_angle\t1")
print("cmd_type\t0")
print("pose_x	pose_y	pose_z	heading")

delta_res = int(resolution/9)
delta = distance / (delta_res * (1 - turn_ratio))
delta_z = (height[1] - height[0]) / (delta_res * (1 - turn_ratio))
x = start_point[0]
y = start_point[1]
z = height[0]
Y = 0 if CCW else np.pi/2
for i in range(resolution):
    if i % delta_res >= delta_res * (1 - turn_ratio):
        Y += (np.pi/2) / (delta_res * turn_ratio) if CCW else -(np.pi/2) / \
                (delta_res * turn_ratio)
    else:
        if i // delta_res == 0 or i // delta_res == 5:
            if CCW:
                x += delta
            else:
                y += delta
        elif i // delta_res == 1 or i // delta_res == 6:
            if CCW:
                y += delta
            else:
                x += delta
        elif i // delta_res == 2 or i // delta_res == 7:
            if CCW:
                x -= delta
            else:
                y -= delta
        elif i // delta_res == 3 or i // delta_res == 8:
            if CCW:
                y -= delta
            else:
                x -= delta
        elif i // delta_res == 4:
            z += delta_z

    while Y > np.pi:
        Y -= 2*np.pi
    while Y < -np.pi:
        Y += 2*np.pi
    if i==0:
        print("{0:.6f}\t{1:.6f}\t{2:.6f}\t{3:.6f}".format(\
                start_point[0],start_point[1],0,Y))
        wr.writerow([x, y, 0, Y])
    print("{0:.6f}\t{1:.6f}\t{2:.6f}\t{3:.6f}".format(x,y,z,Y))
    wr.writerow([x, y, z, Y])
f.close()    
