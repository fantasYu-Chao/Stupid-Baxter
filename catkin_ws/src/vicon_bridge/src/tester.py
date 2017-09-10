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


#define subscriber node
binCor = []
class getBin():
	def __init__(self):
		subNode = rospy.Subscriber("/vicon/Bin/Bin", TransformStamped, self.callback_bin)
		rospy.rostime.set_rostime_initialized( True )


	def callback_bin(self, data):
		binCor = []

		binCor.append(data.transform.translation.x)
		binCor.append(data.transform.translation.y)
		binCor.append(data.transform.translation.z)
		binCor.append(data.transform.rotation.x)
		binCor.append(data.transform.rotation.y)
		binCor.append(data.transform.rotation.z)
		binCor.append(data.transform.rotation.w)
		print binCor


class retrieveSponge():

	def __init__(self):
		subNode = rospy.Subscriber("/vicon/Sponge/Sponge", TransformStamped, self.callback_sponge)
		rospy.rostime.set_rostime_initialized( True )


	def callback_sponge(self, data):
		swx = data.transform.translation.x
		swy = data.transform.translation.y
		swz = data.transform.translation.z
		sqx = data.transform.rotation.x
		sqy = data.transform.rotation.y
		sqz = data.transform.rotation.z
		sqw = data.transform.rotation.w
		print "sponge data: %.4f %.4f %.4f %.4f %.4f %.4f %.4f" % (swx, swy, swz, sqx, sqy, sqz, sqw)

if __name__=='__main__':
    sponge = retrieveSponge()
    binLoc = getBin()
 
    rospy.init_node('Tracker', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "shutting down"

