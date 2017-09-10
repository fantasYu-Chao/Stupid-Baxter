#!/usr/bin/env python

import roslib
import sys
import rospy
import numpy as np
import pdb
from matplotlib import pyplot as plt
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import TransformStamped
import os

class converter():
    def __init__(self):
	print 'try to subscribe from vicon/Bin/Bin'
        self.groundtruth_sub = rospy.Subscriber("/vicon/Bin/Bin", TransformStamped, self.callback_groundtruth)
        rospy.rostime.set_rostime_initialized( True )
        print 'recorder is waiting for Image and Vicon.'


    def callback_groundtruth(self, data):
	wx = data.transform.translation.x
	wy = data.transform.translation.y
	wz = data.transform.translation.z
	qx = data.transform.rotation.x
	qy = data.transform.rotation.y
	qz = data.transform.rotation.z
	qw = data.transform.rotation.w
        groundtruthfile = open('./Data/groundtruth.txt', 'a')	
	time = rospy.rostime.get_time()
        print >> groundtruthfile, "%.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f" %(time, wx, wy, wz, qx, qy, qz, qw)
    
if __name__=='__main__':
    ic = converter()
    rospy.init_node('Recorder', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "shutting down"
	groundtruthfile.close()

