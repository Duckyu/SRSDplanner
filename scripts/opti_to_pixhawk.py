#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Fri Jul 29 00:03:30 2022

@author: mason
"""

''' import libraries '''
import time
import numpy as np

import rospy
from geometry_msgs.msg import PoseStamped

import sys
import signal

def signal_handler(signal, frame): # ctrl + c -> exit program
        print('You pressed Ctrl+C!')
        sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)


''' class '''

class gtpub():
    def __init__(self):
        rospy.init_node('gtgt', anonymous=True)
        self.model_name = rospy.get_param("model_name", 'model')
        self.my_pose = rospy.Subscriber('/vrpn_client_node/'+self.model_name+'/pose', PoseStamped, self.opti_cb)
        self.vision_pose_pub = rospy.Publisher('mavros/vision_pose/pose', PoseStamped, queue_size=1)

        self.rate = rospy.Rate(40)
        self.gt_pose = PoseStamped()
        self.check = 0

    def opti_cb(self, msg):
        self.check = 1
        self.gt_pose = msg

''' main '''
pub_class = gtpub()

if __name__ == '__main__':
    while 1:
        try:
            if pub_class.check == 1:                
                pub_class.vision_pose_pub.publish(pub_class.gt_pose)
                pub_class.check = 0
                pub_class.rate.sleep()
        except (rospy.ROSInterruptException, SystemExit, KeyboardInterrupt) :
            sys.exit(0)
        # except:
        #     print("exception")
        #     pass
