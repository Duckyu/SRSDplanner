#!/usr/bin/env python

import rospy
import tf2_ros
import sys
from packaging import version

import numpy as np
import scipy
from scipy.spatial.transform import Rotation as Rot

from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
from nav_msgs.msg import Odometry

class gtParsePubVision:
    robot_name = "UAV0"
    model_name = "CJH_UAV_0"
    start_gt = Pose()
    gt = Pose()
    local_gt = PoseStamped()
    start_gt_save = False
    sitl_flag = False

    br       = tf2_ros.StaticTransformBroadcaster()
    vis_pub  = rospy.Publisher("mavros/vision_pose/pose", PoseStamped, queue_size=1)
    gt_odom_pub = rospy.Publisher("odom", Odometry, queue_size=1)
    
    def __init__(self, _robot_name, _model_name, _sitl_flag):
        self.robot_name = _robot_name
        self.model_name = _model_name
        self.sitl_flag = _sitl_flag
        self.sub_setup()
    
    def sub_setup(self):
        if self.sitl_flag:
            gt_sub  = rospy.Subscriber("/gazebo/model_states", ModelStates, self.gz_gt_callback)
        else:
            gt_sub = rospy.Subscriber('/vrpn_client_node/'+self.model_name+'/pose', PoseStamped, self.mc_gt_callback)
        
        
    def gz_gt_callback(self, msg):
        self.check_save_start(self.find_model(msg))       

    def mc_gt_callback(self, msg):
        self.check_save_start(msg.pose)

    def find_model(self, _msg):
        name_list = _msg.name
        for idx, robot in enumerate(name_list):
            if robot == self.robot_name:
                pose = _msg.pose[idx]
        return pose

    def check_save_start(self, _pose):
        self.gt = _pose
        if self.start_gt_save:
            self.local_gt = self.gt_to_local(_pose, self.start_gt)
        else:
            self.start_gt_save = True
            self.start_gt = _pose
            rospy.loginfo("{0} start pose saved {1}".format(self.robot_name, self.start_gt))

    def gt_to_local(self, _pose, _start_gt):
        pose_T = self.gt_to_transformation(_pose)
        _start_gt.orientation.x = 0
        _start_gt.orientation.y = 0
        _start_gt.orientation.z = 0
        _start_gt.orientation.w = 1
        start_T = self.gt_to_transformation(_start_gt)

        local_T = np.matmul(np.linalg.inv(start_T), pose_T)
        local = PoseStamped()
        local.header.stamp = rospy.Time.now()
        local.header.frame_id = self.robot_name
        local.pose.position.x = local_T[0,3]
        local.pose.position.y = local_T[1,3]
        local.pose.position.z = local_T[2,3]
        
        if(version.parse(scipy.__version__) > version.parse("1.4.0")):
            local.pose.orientation.x = Rot.from_matrix(local_T[:3,:3]).as_quat()[0]
            local.pose.orientation.y = Rot.from_matrix(local_T[:3,:3]).as_quat()[1]
            local.pose.orientation.z = Rot.from_matrix(local_T[:3,:3]).as_quat()[2]
            local.pose.orientation.w = Rot.from_matrix(local_T[:3,:3]).as_quat()[3]
        else:
            local.pose.orientation.x = Rot.from_dcm(local_T[:3,:3]).as_quat()[0]
            local.pose.orientation.y = Rot.from_dcm(local_T[:3,:3]).as_quat()[1]
            local.pose.orientation.z = Rot.from_dcm(local_T[:3,:3]).as_quat()[2]
            local.pose.orientation.w = Rot.from_dcm(local_T[:3,:3]).as_quat()[3]
        
        return local

    def gt_to_transformation(self, _pose):
        T = np.identity(n=4, dtype=np.float64)
        if(version.parse(scipy.__version__) > version.parse("1.4.0")):
            R = Rot.from_quat([_pose.orientation.x, _pose.orientation.y, _pose.orientation.z, _pose.orientation.w]).as_matrix()
        else:
            R = Rot.from_quat([_pose.orientation.x, _pose.orientation.y, _pose.orientation.z, _pose.orientation.w]).as_dcm()
        t = np.array([_pose.position.x, _pose.position.y, _pose.position.z])
        T[:3,:3] = R
        T[:3,3] = t.T

        return T

    def vision_pub(self):
        self.vis_pub.publish(self.local_gt)
        # rospy.loginfo(gt_vis.local_gt)

    def gt_odometry_pub(self):
        odom = Odometry()
        odom.header.frame_id = "world"
        odom.pose.pose = self.gt
        self.gt_odom_pub.publish(odom)

    def pub_start_tf(self):
        start_trans = TransformStamped()
        start_trans.header.stamp = rospy.Time.now()
        start_trans.header.frame_id = "world"
        start_trans.child_frame_id = self.robot_name
        start_trans.transform.translation.x = self.start_gt.position.x
        start_trans.transform.translation.y = self.start_gt.position.y
        start_trans.transform.translation.z = self.start_gt.position.z
        start_trans.transform.rotation = self.start_gt.orientation
        self.br.sendTransform(start_trans)


if __name__ == '__main__':
    # global pose
    # global setpoint_msg

    rospy.init_node('gt_vision', anonymous=True)
    
    rate = rospy.Rate(40)
    
    robot_name = rospy.get_param("robot_name")
    model_name = rospy.get_param("model_name", 'model')
    sitl_flag = rospy.get_param("/sitl_flag", False)

    gt_vis = gtParsePubVision(robot_name, model_name, sitl_flag)

    while not rospy.is_shutdown():
        try:
            if gt_vis.start_gt_save:
                # gt_vis.pub_start_tf()
                # gt_vis.gt_odometry_pub()
                gt_vis.vision_pub()
                
            rate.sleep()
        except rospy.ROSInterruptException:
            rospy.logerr_once('Node terminated by pressing Ctrl+C!')
            sys.exit(0)
