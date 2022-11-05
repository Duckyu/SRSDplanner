#!/usr/bin/env python

import os
import rospy
import tf2_ros
import sys
import numpy as np
from scipy.spatial.transform import Rotation as Rot
from scipy.spatial import distance

from mavros_msgs.srv import CommandBool, SetMode
from nav_msgs.msg import Path, Odometry
from mavros_msgs.msg import PositionTarget, State, ExtendedState
from geometry_msgs.msg import PoseStamped, TransformStamped

class pathLoader:
    file_path  = "/home/duckyu/ws/duck_base/src/path_generator/path/indoor/path1"
    robot_name  = "UAV0"
    start_pose = [0, 0, 0, 0]
    path_msg = Path()
    ctrl_type = 0
    rot_type = 0
    cmd_type = 0

    ros_version = os.environ.get('ROS_DISTRO')
    
    #publisher
    br       = tf2_ros.StaticTransformBroadcaster()
    local_path_pub = rospy.Publisher("path", Path, queue_size=100)

    def __init__(self, _file_dir, _robot_name):
        self.file_path = _file_dir + "/" + _robot_name + ".txt"
        self.robot_name = _robot_name

    def __str__(self):
        return "Path is generated for {0}\nPath text file located in: {1}".format(self.robot_name, self.file_path)

#    def print(self):
#        rospy.loginfo("Path is generated for {0}".format(self.robot_name))
#        rospy.loginfo("Path text file located in: {0}".format(self.file_path))
    
    def load(self):
        path = []
        try:
            f = open(self.file_path, "r")
        except FileNotFoundError:
            rospy.logfatal("Cannot open file {0}".format(self.file_path))
        data = f.read()
        data_list = data.split("\n")
        control_cfg = data_list[0]
        control_cfg = control_cfg.split("\t")
        control_cfg = self.text_clear(control_cfg, '')
        self.ctrl_type = float(control_cfg[1])

        rot_cfg = data_list[1]
        rot_cfg = rot_cfg.split("\t")
        rot_cfg = self.text_clear(rot_cfg, '')
        self.rot_type = float(rot_cfg[1])

        cmd_type = data_list[2]
        cmd_type = cmd_type.split("\t")
        cmd_type = self.text_clear(cmd_type, '')
        self.cmd_type = float(cmd_type[1])

        data_list = data_list[4:]
        # data_list = data_list[3:]
        for data in data_list:
            line = data.split("\t")
            line = self.text_clear(line, '')
            line = self.convert_to_float(line)
            path.append(line)
        self.start_pose = path[0]
        path = path[1:]
        local_path = []

        empty_list=[]
        path = self.text_clear(path, empty_list)

        for waypoint in path:
            local_path.append(self.convert_to_local(waypoint, self.start_pose[:3]+[0]))
            # local_path.append(self.convert_to_local(waypoint, self.start_pose))

        return local_path

    def convert_to_local(self, _pose, _start_pose):
        pose_T = self.list_to_transformation(_pose)
        start_T = self.list_to_transformation(_start_pose)
        local_pose_T = np.matmul(np.linalg.inv(start_T), pose_T)
        if self.ros_version == 'melodic':
            local_pose = [local_pose_T[0,3], local_pose_T[1,3], local_pose_T[2,3], Rot.from_dcm(local_pose_T[:3,:3]).as_euler('zxy')[0]]
        elif self.ros_version == 'noetic':
            local_pose = [local_pose_T[0,3], local_pose_T[1,3], local_pose_T[2,3], Rot.from_matrix(local_pose_T[:3,:3]).as_euler('zxy')[0]]

        return local_pose

    def list_to_transformation(self, _pose_list):
        T = np.identity(n=4, dtype=np.float64)
        if self.ros_version == 'melodic':
            R = Rot.from_euler('z', _pose_list[3]).as_dcm()
        elif self.ros_version == 'noetic':
            R = Rot.from_euler('z', _pose_list[3]).as_matrix()
        t = np.array(_pose_list[:3])
        T[:3,:3] = R
        T[:3,3] = t.T

        return T

    def convert_to_float(self, _list):
        float_list = []
        for item in _list:
            float_list.append(float(item))
        return float_list
    
    def text_clear(self, _list, _remover):
        while _remover in _list:
            _list.remove(_remover)
        return _list
    
    def pub_path(self):
        self.local_path_pub.publish(self.path_msg)
        
    def generate_path_msg(self, _local_path):
        self.path_msg.header.stamp = rospy.Time()
        self.path_msg.header.frame_id = self.robot_name
        for _pose in _local_path:
            ps = PoseStamped()
            ps.pose.position.x = _pose[0]
            ps.pose.position.y = _pose[1]
            ps.pose.position.z = _pose[2]
            np_ori = Rot.from_euler('z', _pose[3]).as_quat()
            ps.pose.orientation.x = float(np_ori[0])
            ps.pose.orientation.y = float(np_ori[1])
            ps.pose.orientation.z = float(np_ori[2])
            ps.pose.orientation.w = float(np_ori[3])
            self.path_msg.poses.append(ps)

    def pub_start_tf(self):
        start_trans = TransformStamped()
        start_trans.header.stamp = rospy.Time.now()
        start_trans.header.frame_id = "world"
        start_trans.child_frame_id = self.robot_name
        start_trans.transform.translation.x = self.start_pose[0]
        start_trans.transform.translation.y = self.start_pose[1]
        start_trans.transform.translation.z = self.start_pose[2]
        start_ori = Rot.from_euler('z', 0).as_quat()
        start_trans.transform.rotation.x = start_ori[0]
        start_trans.transform.rotation.y = start_ori[1]
        start_trans.transform.rotation.z = start_ori[2]
        start_trans.transform.rotation.w = start_ori[3]
        self.br.sendTransform(start_trans)
