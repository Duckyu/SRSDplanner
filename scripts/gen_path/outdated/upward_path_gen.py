#!/usr/bin/env python
from dis import dis
from tracemalloc import start
import numpy as np
import csv

CCW = False
start_point = [-3.0, 3.0]
height = [3.0, 6.0]
distance = height[1] - height[0]
resolution = 200
turn_ratio = 0.2
iteration = 3

p = 'path/outdoor/path18/'
f = open(p + 'plot_uav5.csv', 'w', newline='')
wr = csv.writer(f)
wr.writerow(['x', 'y', 'z', 'yaw'])

print("control\t0")
print("yaw_angle\t1")
print("cmd_type\t0")
print("pose_x	pose_y	pose_z	heading")

half_res = int(resolution/2)
delta = distance / (half_res * (1 - 2*turn_ratio))
x = start_point[0]
y = start_point[1]
z = height[0]
Y = 0 if CCW else np.pi/2
for j in range(iteration):
    for i in range(resolution):
        Y += (np.pi*2) / half_res if CCW else -(np.pi*2) / half_res
        if i % half_res >= half_res * (1 - turn_ratio) or i % half_res < half_res * turn_ratio:
            z = z
        else:
            if i // half_res == 0:
                z += delta
            elif i // half_res == 1:
                z -= delta
        
        while Y > np.pi:
            Y -= 2*np.pi
        while Y < -np.pi:
            Y += 2*np.pi
        if i==0 and j == 0:
            print("{0:.6f}\t{1:.6f}\t{2:.6f}\t{3:.6f}".format(\
                    start_point[0],start_point[1],0,Y))
            wr.writerow([x, y, 0, Y])
        print("{0:.6f}\t{1:.6f}\t{2:.6f}\t{3:.6f}".format(x,y,z,Y))
        wr.writerow([x, y, z, Y])
f.close()    
