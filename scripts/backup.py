#!/usr/bin/env python3

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
    robot_name  = "uav0"
    start_pose = [0, 0, 0, 0]
    path_msg = Path()
    
    #publisher
    local_path_pub = rospy.Publisher("path", Path, queue_size=100)

    def __init__(self, _file_dir, _robot_name):
        self.file_path = _file_dir + "/" + _robot_name + ".txt"
        self.robot_name = _robot_name

    def print(self):
        rospy.loginfo("Path is generated for {0}".format(self.robot_name))
        rospy.loginfo("Path text file located in: {0}".format(self.file_path))
    
    def load(self):
        path = []
        try:
            f = open(self.file_path, "r")
        except FileNotFoundError:
            rospy.logfatal("Cannot open file {0}".format(self.file_path))
        data = f.read()
        data_list = data.split("\n")
        data_list = data_list[1:]
        for data in data_list:
            line = data.split("\t")
            line = self.text_clear(line, '')
            line = self.convert_to_float(line)
            path.append(line)
        start_list = path[0]
        path = path[1:]
        local_path = []

        empty_list=[]
        path = self.text_clear(path, empty_list)

        for waypoint in path:
            local_path.append(self.convert_to_local(waypoint, start_list))

        return local_path

    def convert_to_local(self, _pose, _start_pose):
        pose_T = self.list_to_transformation(_pose)
        start_T = self.list_to_transformation(_start_pose)
        local_pose_T = np.matmul(np.linalg.inv(start_T), pose_T)
        local_pose = [local_pose_T[0,3], local_pose_T[1,3], local_pose_T[2,3], Rot.from_matrix(local_pose_T[:3,:3]).as_euler('zxy')[0]]

        return local_pose

    def list_to_transformation(self, _pose_list):
        T = np.identity(n=4, dtype=np.float64)
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

class flightControl:

    odom_msg = Odometry()

    #publisher
    control_pub = rospy.Publisher("mavros/setpoint_raw/local", PositionTarget, queue_size=10)
    local_odom_pub = rospy.Publisher("odom", Odometry, queue_size=10)
    
    # mode : 'hover: 0', 'move: 1', 'pure rotation: 2'
    mode = 0
    
    #(trans)_(yaw)_mask
    pos_pos_mask = 0b100111111000
    vel_vel_mask = 0b010111000111
    pos_vel_mask = 0b010111111000
    vel_pos_mask = 0b100111000111

    robot_name = ""

    vel = 2.0
    #yaw_rate = 1.570796
    arrival_dist = 0.05
    start_pose = [0, 0, 0, 0]
    path = []
    target_idx = 0
    target = [0.0, 0.0, 0.0, 0.0]

    setpoint_msg = PositionTarget()
    pose = [0.0, 0.0, 0.0, 0.0]
    state = State()
    ext_state = ExtendedState()

    def __init__(self, _robot_name, _start_pose, _vel, _arrival_dist, _path):#, _yaw_rate
        self.vel = _vel
        #self.yaw_rate = _yaw_rate
        self.path = _path
        self.target = self.path[0]
        self.start_pose = _start_pose
        self.robot_name = _robot_name
        self.arrival_dist = _arrival_dist
        self.sub_setup()

    def path_follow(self):
        self.move(self.target, self.mode)
    
    def sub_setup(self):
        state_sub          = rospy.Subscriber("mavros/state", State, self.state_callback)
        ext_state_sub      = rospy.Subscriber("mavros/extended_state", ExtendedState, self.ext_state_callback)
        local_position_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, self.local_position_callback)

    def state_callback(self, data):  
        self.state = data

    def ext_state_callback(self, data): 
        self.ext_state = data

    def local_position_callback(self, data):
        current_quat = [data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w]
        yaw = Rot.from_quat(current_quat).as_euler('zyx')[0]
        trans = [data.pose.position.x, data.pose.position.y, data.pose.position.z]
        self.pose = trans + [yaw]

        diff,_,_,_ = self.distance_to_target(self.pose, self.target)
        if diff < self.arrival_dist:
            if self.target_idx < len(self.path)-1:
                self.target_idx = self.target_idx + 1
                self.mode = 1
                rospy.loginfo("Going to waypoint {0}/{1}".format(self.target_idx,len(self.path)))
            else:
                self.mode = 0
                rospy.loginfo_once("{0} Path end".format(self.robot_name))
            
            self.target = self.path[self.target_idx]

    def move(self, _target, _mode):
        control_msg = PositionTarget()
        control_msg.coordinate_frame = control_msg.FRAME_LOCAL_NED; #value: 1, EastNorthUp frame FRAME_LOCAL_NED

        if _mode == 0:
            control_msg.type_mask = self.pos_pos_mask
            control_msg.position.x = _target[0]
            control_msg.position.y = _target[1]
            control_msg.position.z = _target[2]
            control_msg.yaw = _target[3]
        elif _mode == 1:
            control_msg.type_mask = self.vel_pos_mask
            vel_cmd = self.pursuit_vel(self.pose, _target)
            control_msg.velocity.x = vel_cmd[0]
            control_msg.velocity.y = vel_cmd[1]
            control_msg.velocity.z = vel_cmd[2]
            control_msg.yaw = _target[3]
        elif _mode == 2:
            control_msg.type_mask = self.pos_vel_mask
            vel_cmd = self.pursuit_vel(self.pose, _target)
            control_msg.position.x = vel_cmd[0]
            control_msg.position.y = vel_cmd[1]
            control_msg.position.z = vel_cmd[2]
            control_msg.yaw_rate = _target[3]
        elif _mode == 3:
            control_msg.type_mask = self.vel_vel_mask
            vel_cmd = self.pursuit_vel(self.pose, _target)
            control_msg.velocity.x = vel_cmd[0]
            control_msg.velocity.y = vel_cmd[1]
            control_msg.velocity.z = vel_cmd[2]
            control_msg.yaw_rate = _target[3]
        else:
            rospy.logerr("Mode set wrong")
            control_msg.type_mask = self.pos_pos_mask
            control_msg.position.x = self.pose[0]
            control_msg.position.y = self.pose[1]
            control_msg.position.z = self.pose[2]
            control_msg.yaw = self.pose[3]
        
        self.control_pub.publish(control_msg)

    def pursuit_vel(self, _pose, _target):
        (_, grad_x, grad_y, grad_z) = self.distance_to_target(_pose, _target)
        velocity_command = [self.vel*grad_x, self.vel*grad_y, self.vel*grad_z] # , self.yaw_rate
        return velocity_command
        

    def distance_to_target(self, _pose, _target):
        dist = distance.euclidean(_pose[:3], _target[:3])
        grad_x = (_target[0] - _pose[0])/dist
        grad_y = (_target[1] - _pose[1])/dist
        grad_z = (_target[2] - _pose[2])/dist
        return (dist, grad_x, grad_y, grad_z)

    def takeoff(self):
        rospy.loginfo("Attempting takeoff")
        while self.state.armed == False or self.pose[2] < 0.15:
            try:
                self.move(self.target, 0)
                arming_srv  = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
                arming_srv(True)
                mode_srv    = rospy.ServiceProxy('mavros/set_mode', SetMode)
                mode_srv(0,"OFFBOARD")
                rospy.loginfo_once("Arming & setMode complete")
            except:
                rospy.logerr_once("Check mavros")

    def generate_odom_msg(self, _pose):
        self.odom_msg.header.stamp = rospy.Time()
        self.odom_msg.header.frame_id = self.robot_name
        self.odom_msg.pose.pose.position.x = _pose[0]
        self.odom_msg.pose.pose.position.y = _pose[1]
        self.odom_msg.pose.pose.position.z = _pose[2]
        np_ori = Rot.from_euler('z', _pose[3]).as_quat()
        self.odom_msg.pose.pose.orientation.x = float(np_ori[0])
        self.odom_msg.pose.pose.orientation.y = float(np_ori[1])
        self.odom_msg.pose.pose.orientation.z = float(np_ori[2])
        self.odom_msg.pose.pose.orientation.w = float(np_ori[3])

    def pub_odom(self):
        self.generate_odom_msg(self.pose)
        self.local_odom_pub.publish(self.odom_msg)
        
if __name__ == '__main__':

    rospy.init_node('vel_pursuit_guide', anonymous=True)
    
    rate = rospy.Rate(60)
    
    file_dir = rospy.get_param("/file_dir")
    robot_name = rospy.get_param("robot_name")

    pl = pathLoader(file_dir, robot_name)
    pl.print()
    path = pl.load()
    pl.generate_path_msg(path)
    
    vel = rospy.get_param("vel")
    #yaw_rate = rospy.get_param("yaw_rate")
    arv_dist = rospy.get_param("arv_dist")
    fc = flightControl(robot_name, pl.start_pose, vel, arv_dist, path)#, yaw_rate
    
    fc.takeoff()

    while not rospy.is_shutdown():
        try:
            fc.path_follow()
            fc.pub_odom()
            pl.pub_path()
            rate.sleep()
        except rospy.ROSInterruptException:
            rospy.loginfo('Node terminated by pressing Ctrl+C!')
            sys.exit(0)
