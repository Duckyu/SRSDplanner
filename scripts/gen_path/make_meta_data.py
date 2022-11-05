#!/usr/bin/env python
import numpy as np
import csv
import argparse

def parse_args():
    parser = argparse.ArgumentParser(
        description='Generate outdoor meta data')
    parser.add_argument('--main_path', 
        type=str,
        default='/home/dklee/uav_ws/src/uav_path_tool/path/outdoor/')
    args = parser.parse_args()
    return args

def main():
    args = parse_args()
    main_path = args.main_path
    meta_csv = open(main_path + 'meta_data.csv', 'w')
    wr = csv.writer(meta_csv)
    wr.writerow(['path', 'type', 'uav', 'x', 'y', 'yaw'])
    
    # spiral, square, infinite, sin
    ## path 1
    path = 1
    wr.writerow([path, 'spiral', 1,    0.0,    0.0,    0])
    wr.writerow([path, 'spiral', 2,    5.0,    0.0,    0])
    wr.writerow([path, 'spiral', 3,    10.0,   0.0,    0])
    wr.writerow([path, 'spiral', 4,    0.0,    5.0,    0])
    wr.writerow([path, 'spiral', 5,    0.0,   10.0,    0])
    # CW =     [True, True, True, True, True]
    # radius = [ 5.0,  5.0,  5.0,  5.0,  5.0]
    # height = [[3.0, 10.0],
    #           [10.0, 3.0],
    #           [3.0, 10.0],
    #           [10.0, 3.0],
    #           [3.0, 10.0]]
    # heading = ['none','none','none','none','none']
    # --iter 3

    ## path 2
    path = 2
    wr.writerow([path, 'spiral', 1,    0.0,    0.0,    0])
    wr.writerow([path, 'spiral', 2,    5.0,    0.0,    -45])
    wr.writerow([path, 'spiral', 3,    10.0,   10.0,    90])
    wr.writerow([path, 'spiral', 4,    0.0,    5.0,    -90])
    wr.writerow([path, 'spiral', 5,    0.0,   10.0,    45])
    # CW =     [True, True, True, True, True]
    # radius = [ 5.0,  5.0,  5.0,  5.0,  5.0]
    # height = [[3.0, 10.0],
    #           [10.0, 3.0],
    #           [3.0, 10.0],
    #           [10.0, 3.0],
    #           [3.0, 10.0]]
    # heading = ['none','none','none','none','none']
    # --iter 3

    ## path 3
    path = 3
    wr.writerow([path, 'spiral', 1,    0.0,    0.0,    0])
    wr.writerow([path, 'spiral', 2,    0.0,    15.0,    180])
    wr.writerow([path, 'spiral', 3,    15.0,   15.0,    0])
    wr.writerow([path, 'spiral', 4,    15.0,    0.0,    180])
    wr.writerow([path, 'spiral', 5,    7.5,   0.0,    0])
    # CW =     [True, True, True, True, True]
    # radius = [ 5.0,  5.0,  5.0,  5.0,  5.0]
    # height = [[3.0, 10.0],
    #           [10.0, 3.0],
    #           [3.0, 10.0],
    #           [10.0, 3.0],
    #           [3.0, 10.0]]
    # heading = ['none','none','none','none','none']
    # --iter 3
    
    # ##path 4
    path = 4
    wr.writerow([path, 'square', 1,    0.0,    0.0,    0])
    wr.writerow([path, 'square', 2,    0.0,    5.0,    45])
    wr.writerow([path, 'square', 3,    5.0,    0.0,    90])
    wr.writerow([path, 'square', 4,    5.0,    5.0,    -45])
    wr.writerow([path, 'square', 5,    10.0,   0.0,    -90])
    # ## path 5
    path = 5
    wr.writerow([path, 'infinite', 1,    0.0,    0.0,    0.0])
    wr.writerow([path, 'infinite', 2,    0.0,    18.0,    0.0])
    wr.writerow([path, 'infinite', 3,    18.0,    0.0,    0.0])
    wr.writerow([path, 'infinite', 4,    18.0,    18.0,    0.0])
    wr.writerow([path, 'infinite', 5,    9.0,   9.0,    0.0])
    # ## path 6
    path = 6
    wr.writerow([path, 'infinite', 1,    0.0,    0.0,    45])
    wr.writerow([path, 'infinite', 2,    0.0,    15.0,    -45])
    wr.writerow([path, 'infinite', 3,    15.0,    0.0,    -45])
    wr.writerow([path, 'infinite', 4,    15.0,    15.0,    45])
    wr.writerow([path, 'infinite', 5,    7.5,   7.5,    0.0])
    # ## path 5
    path = 7
    wr.writerow([path, 'infinite', 1,    0.0,    0.0,    0.0])
    wr.writerow([path, 'infinite', 2,    0.0,    18.0,    0.0])
    wr.writerow([path, 'infinite', 3,    18.0,    0.0,    0.0])
    wr.writerow([path, 'infinite', 4,    18.0,    18.0,    0.0])
    wr.writerow([path, 'infinite', 5,    9.0,   9.0,    0.0])
    # ## path 8
    path = 8
    wr.writerow([path, 'sin', 1,    0.0,    0.0,    0.0])
    wr.writerow([path, 'sin', 2,    0.0,    10.0,    0.0])
    wr.writerow([path, 'sin', 3,    10.0,    0.0,    0.0])
    wr.writerow([path, 'sin', 4,    10.0,    10.0,    0.0])
    wr.writerow([path, 'sin', 5,    -10.0,   0.0,    0.0])
    # ## path 9
    path = 9
    wr.writerow([path, 'sin', 1,    4.0,    0.0,    0.0])
    wr.writerow([path, 'sin', 2,    0.0,    4.0,    90])
    wr.writerow([path, 'sin', 3,    -4.0,    0.0,    180])
    wr.writerow([path, 'sin', 4,    0.0,    -4.0,    -90])
    wr.writerow([path, 'spiral', 5,    0.0,   -1.5,    0.0])
    # amplitude =  [ 3.0,  3.0,  3.0,  3.0,  3.0] # + : CW, - : CCW
    # period = [ 0.5,  0.5,  0.5,  0.5,  0.5]
    # use_yz = [False, False, False, False, False]
    # no_turn = [False, False, False, False, False]
    # height = [
    #     [3.0, 6.0],
    #     [3.0, 6.0],
    #     [3.0, 6.0],
    #     [3.0, 6.0],
    #     [3.0, 10.0]]
    # ## path 10
    path = 10
    wr.writerow([path, 'square', 1,    0.0,    0.0,    0.0])
    wr.writerow([path, 'square', 2,    0.0,    10.0,    -90])
    wr.writerow([path, 'square', 3,    10.0,    0.0,    90])
    wr.writerow([path, 'square', 4,    10.0,    10.0,    180])
    wr.writerow([path, 'square', 5,    13.0,   13.0,    -90])
    # CW =        [False, False, False, False, True]
    # fix_turn = [True, True, False, False, False]
    # step_z = [True, True, True, True, True]
    # distance =  [ 10.0,  10.0,  10.0,  10.0,  16.0]
    # height = [
    #     [3.0, 10.0],
    #     [3.0, 10.0],
    #     [3.0, 10.0],
    #     [3.0, 10.0],
    #     [10.0, 5.0]]

    ## path 11
    path = 11
    wr.writerow([path, 'spiral', 1,    0.0,    0.0,    0])
    wr.writerow([path, 'spiral', 2,    10.0,    0.0,    0])
    wr.writerow([path, 'spiral', 3,    20.0,   0.0,    0])
    wr.writerow([path, 'spiral', 4,    5.0,    8.0,    0])
    wr.writerow([path, 'spiral', 5,    15.0,   8.0,    0])
    # CW =     [True, True, True, True, True]
    # radius = [ 4.0,  4.0,  4.0,  4.0,  4.0]
    # height = [[3.0, 3.0],
    #           [3.0, 3.0],
    #           [3.0, 3.0],
    #           [3.0, 3.0],
    #           [3.0, 3.0]]
    # heading = ['none','none','none','none','none']
    # --iter 3
    meta_csv.close()


if __name__ == '__main__':
    main()