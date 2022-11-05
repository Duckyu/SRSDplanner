#!/usr/bin/env python

import rospy
import numpy as np
from scipy.spatial.transform import Rotation as Rot
from scipy.spatial import distance

from mavros_msgs.srv import CommandBool, SetMode
from std_msgs.msg import Empty, String
from nav_msgs.msg import Path, Odometry
from mavros_msgs.msg import PositionTarget, State, ExtendedState
from geometry_msgs.msg import PoseStamped, TransformStamped

class flightControl:

    odom_msg = Odometry()

    #publisher
    control_pub = rospy.Publisher("mavros/setpoint_raw/local", PositionTarget, queue_size=10)
    local_odom_pub = rospy.Publisher("odom", Odometry, queue_size=10)
    log_pub = rospy.Publisher("log", String, queue_size=10)
    
    # mode : 'hover: 0', 'move: 1', 'pure rotation: 2', 'velocity velocity: 3'
    mode = 0
    # ctrl_type : 'vel pursuit: 0', 'LoS: 1', 'position: 2'
    ctrl_type = 0
    # ctrl_type : 'yaw target: 0', 'yaw vel: 1''
    rot_type = 0
    # cmd_type : 'w/o command topic: 0', 'w/ command topic: 1''
    cmd_type = 0
    
    #(trans)_(yaw)_mask
    pos_pos_mask = 0b100111111000
    vel_vel_mask = 0b010111000111
    pos_vel_mask = 0b010111111000
    vel_pos_mask = 0b100111000111

    robot_name = ""

    takeoff_flag = False
    start = False
    next = False
    vel = 2.0
    yaw_rate = 0.1570796
    arrival_dist = 0.05
    arrival_yaw = 0.2
    r_los = 1.0
    start_pose = [0, 0, 0, 0]
    path = []
    target_idx = 0
    past_target = [0.0, 0.0, 0.0, 0.0]
    target = [0.0, 0.0, 0.0, 0.0]

    setpoint_msg = PositionTarget()
    pose = [0.0, 0.0, 0.0, 0.0]
    state = State()
    ext_state = ExtendedState()

    def __init__(self, _robot_name, _start_pose, _vel, _arrival_dist, _arrival_yaw, _path, _ctrl_type = 1, _rot_type = 0, _cmd_type = 0, _r_los = 1.0, _yaw_rate = 0.1570796):#
        self.vel = _vel
        self.yaw_rate = _yaw_rate
        self.path = _path
        self.target = self.path[0]
        self.start_pose = _start_pose
        self.past_target = _start_pose
        self.robot_name = _robot_name
        self.arrival_dist = _arrival_dist
        self.arrival_yaw = _arrival_yaw
        self.r_los  = _r_los
        self.ctrl_type = _ctrl_type
        self.rot_type = _rot_type
        self.cmd_type = _cmd_type
        self.sub_setup()

    def path_follow(self):
        self.move(self.target, self.mode)
    
    def sub_setup(self):
        state_sub          = rospy.Subscriber("mavros/state", State, self.state_callback)
        ext_state_sub      = rospy.Subscriber("mavros/extended_state", ExtendedState, self.ext_state_callback)
        local_position_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, self.local_position_callback)
        takeoff_sub = rospy.Subscriber("/takeoff", Empty, self.takeoff_callback)
        start_sub = rospy.Subscriber("/start", Empty, self.start_callback)
        next_sub = rospy.Subscriber("/next", Empty, self.next_callback)

    def state_callback(self, data):  
        self.state = data

    def ext_state_callback(self, data): 
        self.ext_state = data

    def local_position_callback(self, data):
        current_quat = [data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w]
        yaw = Rot.from_quat(current_quat).as_euler('zyx')[0]
        trans = [data.pose.position.x, data.pose.position.y, data.pose.position.z]
        self.pose = trans + [yaw]

        diff,_,_,_,yaw_diff = self.distance_to_target(self.pose, self.target)
        self.log_pub.publish(
            '''----------------
            next:     {0}
            control:  {1:01d}
            mode:     {2:01d}
            diff:     {3:0.4f}
            yaw diff: {4:0.4f}
            target:   {5:03d}
            ----------------'''.format(self.next,int(self.ctrl_type),int(self.mode),diff,yaw_diff,int(self.target_idx)))
        
        if self.ctrl_type == 1:
            threshold = self.r_los+0.1
        else:
            threshold = self.arrival_dist
        
        if diff < threshold and abs(yaw_diff) < self.arrival_yaw:
            if self.target_idx == 0:
                while(not self.start):
                    rospy.loginfo_once("Waiting start topic")
                rospy.loginfo("Start mission")
            if self.target_idx < len(self.path)-1:
                if self.cmd_type == 0 or (self.cmd_type == 1 and self.next):
                    self.target_idx = self.target_idx + 1
                    if self.rot_type == 0:
                        if self.ctrl_type == 2:
                            self.mode = 0
                        else:
                            self.mode = 1
                    elif self.rot_type == 1:
                        if self.ctrl_type == 2:
                            self.mode = 2
                        else:
                            self.mode = 3
                    rospy.loginfo("{2} going to waypoint {0}/{1}".format(self.target_idx,len(self.path),self.robot_name))
                    self.next = False
                elif self.cmd_type == 1 and not self.next:
                    rospy.loginfo("{0} waiting for next command".format(self.robot_name))
                    self.mode = 0
                self.past_target = self.target
                self.target = self.path[self.target_idx]
            else:
                self.mode = 0
                rospy.loginfo_once("{0} Path end".format(self.robot_name))
            
            # rospy.loginfo("{0} past/target: {1}/{2}".format(self.robot_name, self.past_target,self.target))
        elif diff < threshold and abs(yaw_diff) > self.arrival_yaw:
            self.mode = 2
            rospy.loginfo("Yaw aligning")

    def takeoff_callback(self, data):  
        self.takeoff_flag = True

    def start_callback(self, data):  
        self.start = True

    def next_callback(self, data):
        diff,_,_,_,yaw_diff = self.distance_to_target(self.pose, self.target)
        if self.ctrl_type == 1:
            threshold = self.r_los
        else:
            threshold = self.arrival_dist
        if diff < threshold and abs(yaw_diff) < self.arrival_yaw:
            self.next = True
        else:
            rospy.logerr("Wait! {0} not arrived yet. next: {1}".format(self.robot_name, self.next))

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
            if self.ctrl_type == 0:
                vel_cmd = self.pursuit_vel(self.pose, _target)
            elif self.ctrl_type == 1:
                vel_cmd = self.los_vel(self.pose, self.past_target, _target, self.r_los)
            elif self.ctrl_type == 2:
                rospy.logerr("Wrong control type")
                self.mode = 0
            try:
                control_msg.velocity.x = vel_cmd[0]
                control_msg.velocity.y = vel_cmd[1]
                control_msg.velocity.z = vel_cmd[2]
                control_msg.yaw = _target[3]
            except:
                rospy.logerr("Wait")
        elif _mode == 2:
            control_msg.type_mask = self.pos_vel_mask
            control_msg.position.x = _target[0]
            control_msg.position.y = _target[1]
            control_msg.position.z = _target[2]
            control_msg.yaw_rate = self.trap_yaw(self.yaw_rate)
        elif _mode == 3:
            control_msg.type_mask = self.vel_vel_mask
            if self.ctrl_type == 0:
                vel_cmd = self.pursuit_vel(self.pose, _target)
            elif self.ctrl_type == 1:
                vel_cmd = self.los_vel(self.pose, self.past_target, _target, self.r_los)
            elif self.ctrl_type == 2:
                rospy.logerr("Wrong control type")
                self.mode = 2
            try:
                control_msg.velocity.x = vel_cmd[0]
                control_msg.velocity.y = vel_cmd[1]
                control_msg.velocity.z = vel_cmd[2]
                control_msg.yaw_rate = self.trap_yaw(self.yaw_rate)
            except:
                rospy.logerr("Wait")                
        else:
            rospy.logerr("Mode set wrong")
            control_msg.type_mask = self.pos_pos_mask
            control_msg.position.x = self.pose[0]
            control_msg.position.y = self.pose[1]
            control_msg.position.z = self.pose[2]
            control_msg.yaw = self.pose[3]
        
        self.control_pub.publish(control_msg)

    def pursuit_vel(self, _pose, _target):
        (_, grad_x, grad_y, grad_z, _) = self.distance_to_target(_pose, _target)
        velocity_command = [self.vel*grad_x, self.vel*grad_y, self.vel*grad_z] # , self.yaw_rate
        return velocity_command
    
    def los_vel(self, _pose, _past_target, _target, _r_los = 1.0):
        (temp_dist, vec_x, vec_y, vec_z, _) = self.distance_to_target(_past_target, _target)
        # (_, vec_x, vec_y, vec_z, _) = self.distance_to_target(_past_target, _target)
        (pp_d, pp_x, pp_y, pp_z, _) = self.distance_to_target(_past_target, _pose)
        perp_d = pp_d * np.sin(np.arccos(np.dot(np.array([vec_x, vec_y, vec_z]), np.array([pp_x, pp_y, pp_z]))))
        local_dist = (float(pp_d)**2 - float(perp_d)**2)**0.5
        rospy.loginfo("residual distance: {0}".format(temp_dist - pp_d))
        
        perp = [[_past_target[0], _past_target[1], _past_target[2]][i] + [vec_x*local_dist, vec_y*local_dist, vec_z*local_dist][i] for i in range(3)]
        if perp_d < _r_los:
            local_los_dist = (_r_los**2 - float(perp_d)**2)**0.5
            local_target = [perp[i] + [vec_x*local_los_dist, vec_y*local_los_dist, vec_z*local_los_dist][i] for i in range(3)]
        else:
            local_target = perp
        local_target.append(self.target[3])
        (_, local_x, local_y, local_z, _) = self.distance_to_target(_pose, local_target)
        
        rospy.loginfo("local_target: {0} {1} {2}".format(local_target[0],local_target[1],local_target[2]))
        # rospy.logerr_once(local_target)
        velocity_command = [self.vel*local_x, self.vel*local_y, self.vel*local_z] # , self.yaw_rate
        return velocity_command
        
    def trap_yaw(self, _yaw_rate):
        (_, _, _, _, diff_yaw) = self.distance_to_target(self.pose, self.target)

        if diff_yaw > _yaw_rate*2:
            # yaw_rate_cmd = -_yaw_rate
            yaw_rate_cmd = _yaw_rate
        elif diff_yaw > _yaw_rate:
            # yaw_rate_cmd = -_yaw_rate*(diff_yaw/_yaw_rate)/2
            yaw_rate_cmd = _yaw_rate*(diff_yaw/_yaw_rate)/2
        elif diff_yaw > 0:
            if self.ctrl_type ==  2:
                self.mode = 2
            else:
                self.mode = 1
            # yaw_rate_cmd = -_yaw_rate/2
            yaw_rate_cmd = _yaw_rate/2
        elif diff_yaw > -_yaw_rate:
            if self.ctrl_type ==  2:
                self.mode = 2
            else:
                self.mode = 1
            # yaw_rate_cmd = _yaw_rate/2
            yaw_rate_cmd = -_yaw_rate/2
        elif diff_yaw > -_yaw_rate*2:
            # yaw_rate_cmd = -_yaw_rate*(diff_yaw/_yaw_rate)/2
            yaw_rate_cmd = _yaw_rate*(diff_yaw/_yaw_rate)/2
        else:
            # yaw_rate_cmd = _yaw_rate
            yaw_rate_cmd = -_yaw_rate

        # rospy.logerr("diff_yaw: {0}, yaw_rate_cmd: {1} ".format(diff_yaw, yaw_rate_cmd))
        
        return yaw_rate_cmd

    def distance_to_target(self, _pose, _target):
        dist = distance.euclidean(_pose[:3], _target[:3])
        dist += 0.001
        # print("{0} dist2target: {1}".format(self.robot_name, dist))
        grad_x = (_target[0] - _pose[0])/dist
        grad_y = (_target[1] - _pose[1])/dist
        grad_z = (_target[2] - _pose[2])/dist
        yaw_diff = _target[3] - _pose[3]
        while yaw_diff < -np.pi:
            yaw_diff += np.pi*2
        while yaw_diff > np.pi:
            yaw_diff -= np.pi*2
        return (dist, grad_x, grad_y, grad_z, yaw_diff)

    def takeoff(self):
        print(self.takeoff_flag)
        while(not self.takeoff_flag):
            rospy.loginfo_once("Waiting takeoff topic")
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