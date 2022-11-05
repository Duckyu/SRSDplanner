#!/usr/bin/env python
import numpy as np
import csv
import argparse

def yaw_sat(input):
    while input > np.pi:
        input -= 2*np.pi
    while input < -np.pi:
        input += 2*np.pi
    return input

def rotate(x, y, base_x, base_y, angle):
    xx = np.cos(angle) * (x - base_x) - np.sin(angle) * (y - base_y) + base_x
    yy = np.sin(angle) * (x - base_x) + np.cos(angle) * (y - base_y) + base_y
    return xx, yy

def get_distance(p1, p2):
    # p1 = np.array((x1, y1, z1))
    # p2 = np.array((x2, y2, z2))
    dis = np.linalg.norm(p1 - p2)
    return dis

def parse_args():
    parser = argparse.ArgumentParser(
        description='Generate infinite path')
    parser.add_argument('path_num', help='number of the path')
    parser.add_argument('--main_path', 
        type=str,
        default='/home/dklee/uav_ws/src/uav_path_tool/path/outdoor/')
    parser.add_argument('--resolution', default=200,
        help='uav speed')
    parser.add_argument('--iter', type=int, default=3,
        help='iteration path')
    parser.add_argument('--collision_th', type=float, default=1.5,
        help='collision check threshold')
    args = parser.parse_args()
    return args

def main():
    args = parse_args()
    path_num = args.path_num
    main_path = args.main_path
    collision_th = args.collision_th
    meta_csv = open(main_path + 'meta_data.csv', 'r')
    rdr = csv.reader(meta_csv)
    uav_setup_l = []
    isInfinite = [False, False, False, False, False]
    for line in rdr:
        if line[0] == path_num:
            if line[1] == 'infinite':
                isInfinite[int(line[2])-1] = True
            uav_setup_l.append(line[1:]) # type, uav, x, y, yaw
    uav_size = len(uav_setup_l)
    if uav_size > 5:
        print("maximum 5 ERROR!!!!!")
        exit()
    print('==============')
    print('The number of uav is {}'.format(uav_size))
    print('==============')
    resolution = args.resolution
    iter = args.iter
    origin_x = []; origin_y = []; origin_yaw = []
    for i, uav in enumerate(uav_setup_l):
        origin_yaw.append(yaw_sat(float(uav[4]) * np.pi / 180.0))
        origin_x.append(float(uav[2]))
        origin_y.append(float(uav[3]))
    ################################################################
    ## you only need to edit this part variables ###################
    ################################################################
    alpha =  [ 5.0,  5.0,  5.0,  5.0,  5.0] # + : CW, - : CCW
    no_turn = [True, True, False, False, True]
    height = [
        [3.0, 15.0],
        [15.0, 3.0],
        [15.0, 3.0],
        [3.0, 15.0],
        [3.0, 15.0]]
    ###########################################
    ### Don't edit below code #################
    ###########################################
    for l in range(uav_size):
        if not any(isInfinite):
            print("no available square path")
            exit()
        elif not isInfinite[l]:
            print("UAV{} is not infinite path".format(l+1))
            continue
        out_csv = open(main_path + 'path{}/'.format(path_num) + 
                    'plot_uav{}.csv'.format(l+1), 'w', newline='')
        out_txt = open(main_path + 'path{}/'.format(path_num) +
                    'UAV{}.txt'.format(l+1), 'w')
        wr = csv.writer(out_csv)
        wr.writerow(['x', 'y', 'z', 'yaw'])
        out_txt.write("control\t0\n")
        out_txt.write("yaw_angle\t1\n")
        out_txt.write("cmd_type\t0\n")
        out_txt.write("pose_x	pose_y	pose_z	heading\n")
        z = height[l][0]
        Y = yaw_sat(origin_yaw[l])
        delta_z = ((height[l][1]-height[l][0])) / \
                float(resolution * iter)
        for j in range(iter):
            for i in range(resolution):
                angle = yaw_sat(2*np.pi/float(resolution)*i + np.pi/2)
                x_ = origin_x[l] + alpha[l] * np.sqrt(2) * np.cos(angle) / \
                    (np.power(np.sin(angle),2) + 1)
                y_ = origin_y[l] + alpha[l] * np.sqrt(2) * np.cos(angle) * \
                    np.sin(angle) / (np.power(np.sin(angle),2) + 1)
                x, y = rotate(x_, y_, origin_x[l], origin_y[l], origin_yaw[l])
                z += delta_z
                if not no_turn[l]:
                    Y += 4 * np.pi / resolution
                if i==0 and j == 0:
                    out_txt.write("{0:.6f}\t{1:.6f}\t{2:.6f}\t{3:.6f}\n".format(x,y,0,Y))
                    wr.writerow([x, y, 0, Y])
                out_txt.write("{0:.6f}\t{1:.6f}\t{2:.6f}\t{3:.6f}\n".format(x,y,z,Y))
                wr.writerow([x, y, z, Y])
        out_csv.close()
        out_txt.close()

    ## collision check
    
    out_1 = open(main_path + 'path{}/'.format(path_num) + 
                'plot_uav{}.csv'.format(1), 'r')
    out_2 = open(main_path + 'path{}/'.format(path_num) + 
                'plot_uav{}.csv'.format(2), 'r')
    out_3 = open(main_path + 'path{}/'.format(path_num) + 
                'plot_uav{}.csv'.format(3), 'r')
    out_4 = open(main_path + 'path{}/'.format(path_num) + 
                'plot_uav{}.csv'.format(4), 'r')
    out_5 = open(main_path + 'path{}/'.format(path_num) + 
                'plot_uav{}.csv'.format(5), 'r')
    rdr_1 = csv.reader(out_1)
    rdr_2 = csv.reader(out_2)
    rdr_3 = csv.reader(out_3)
    rdr_4 = csv.reader(out_4)
    rdr_5 = csv.reader(out_5)

    cause = []
    ground_th = 2.0
    for j, (l1, l2, l3, l4, l5) in \
            enumerate(zip(rdr_1, rdr_2, rdr_3, rdr_4, rdr_5)):
        if l1[0] == 'x':
            continue
        cause = []
        p1 = np.array((float(l1[0]), float(l1[1]), float(l1[2])))
        p2 = np.array((float(l2[0]), float(l2[1]), float(l2[2])))
        p3 = np.array((float(l3[0]), float(l3[1]), float(l3[2])))
        p4 = np.array((float(l4[0]), float(l4[1]), float(l4[2])))
        p5 = np.array((float(l5[0]), float(l5[1]), float(l5[2])))
        p1_g = np.array((float(l1[0]), float(l1[1]), 0.0))
        p2_g = np.array((float(l2[0]), float(l2[1]), 0.0))
        p3_g = np.array((float(l3[0]), float(l3[1]), 0.0))
        p4_g = np.array((float(l4[0]), float(l4[1]), 0.0))
        p5_g = np.array((float(l5[0]), float(l5[1]), 0.0))
        if j > 1:
            if get_distance(p1, p1_g) < ground_th:
                cause.append([0, 1])
            if get_distance(p2, p2_g) < ground_th:
                cause.append([0, 2])
            if get_distance(p3, p3_g) < ground_th:
                cause.append([0, 3])
            if get_distance(p4, p4_g) < ground_th:
                cause.append([0, 4])
            if get_distance(p5, p5_g) < ground_th:
                cause.append([0, 5])

        if get_distance(p1, p2) < collision_th:
            cause.append([1, 2])
        if get_distance(p1, p3) < collision_th:
            cause.append([1, 3])
        if get_distance(p1, p4) < collision_th:
            cause.append([1, 4])
        if get_distance(p1, p5) < collision_th:
            cause.append([1, 5])
        if get_distance(p2, p3) < collision_th:
            cause.append([2, 3])
        if get_distance(p2, p4) < collision_th:
            cause.append([2, 4])
        if get_distance(p2, p5) < collision_th:
            cause.append([2, 5])
        if get_distance(p3, p4) < collision_th:
            cause.append([3, 4])
        if get_distance(p3, p5) < collision_th:
            cause.append([3, 5])
        if get_distance(p4, p5) < collision_th:
            cause.append([4, 5])
        if not len(cause) == 0:
            for c in cause:
                print("between path {}, {} / index {}".format(c[0], c[1], j))

if __name__ == '__main__':
    main()

