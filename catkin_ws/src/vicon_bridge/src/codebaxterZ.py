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
import argparse
import struct

# baxter_interface - Baxter Python API
import baxter_interface

#initialize node
rospy.init_node('Pick_Place')

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

binCor = []
spongeCor = []

# Fix to a given orientation
orient=[
          0.6995856718498753,
          0.006060723615985742,
          0.7140665212801541,
          -0.025537395295301586,
       ]

bintranslate=[]
spongetranslate=[]

# Setting Baxter limbs and calibrating grippers
rarm = baxter_interface.Limb('right')
rgripper = baxter_interface.Gripper('right')
rgripper.calibrate()

larm = baxter_interface.Limb('left')
lgripper = baxter_interface.Gripper('left')
lgripper.calibrate()   

#------------------------------------------------------------------------------
#                                class getBin()
#
# This class subscribes to the VICON node and retrieves the coordinates of the
# bin. The name should match the name of one of the models that VICON is
# publishing. In this class the exact name is "bin". VICON has to be connected
# and running 
#
#------------------------------------------------------------------------------
class getBin():
	def __init__(self):

		subNode = rospy.Subscriber("/vicon/Bin/Bin", TransformStamped, self.callback_bin)
		rospy.rostime.set_rostime_initialized( True )

	def callback_bin(self, data):
		#Clean list
                del binCor[:]

                #Write current coordinates
		binCor.append(data.transform.translation.x)
		binCor.append(data.transform.translation.y)
		binCor.append(data.transform.translation.z)
		binCor.append(data.transform.rotation.x)
		binCor.append(data.transform.rotation.y)
		binCor.append(data.transform.rotation.z)
		binCor.append(data.transform.rotation.w)

#------------------------------------------------------------------------------
#                                class getBin()
#
# This class subscribes to the VICON node and retrieves the coordinates of the
# object (sponge).The name should match the name of one of the models that
# VICON is publishing. In this class the exact name is "sponge". VICON has to
# be connected and running 
#
#------------------------------------------------------------------------------
class retrieveSponge():
	def __init__(self):
		subNode = rospy.Subscriber("/vicon/Sponge/Sponge", TransformStamped, self.callback_sponge)
		rospy.rostime.set_rostime_initialized( True )

	def callback_sponge(self, data):
                #Clean list
                del spongeCor[:]

                #Write current coordinates
		spongeCor.append(data.transform.translation.x)
		spongeCor.append(data.transform.translation.y)
		spongeCor.append(data.transform.translation.z)
		spongeCor.append(data.transform.rotation.x)
		spongeCor.append(data.transform.rotation.y)
		spongeCor.append(data.transform.rotation.z)
		spongeCor.append(data.transform.rotation.w)
        
#------------------------------------------------------------------------------
#                          get_angles(limb,X)
#
# This function fins the joint angles that Baxter's arm should do to reach a
# point X (x,y,z). The function uses Baxter's inverse kinematics software.
# This software has its limitations regarding the solution. The main problem
# found is that it may not find the optimal path, or not find one at all, even
# if there is one.  
#
# inputs
#     limb: 'left' or 'right'. to respect to which arm calculate the angles.
#     X: [x,y,z]. Coordinates in respect to Baxter's origin. This is the point
#        Baxter will reach.
# 
#------------------------------------------------------------------------------
def get_angles(limb,X):
    ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    ikreq = SolvePositionIKRequest()
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    poses = {
        limb: PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(
                    x=X[0],
                    y=X[1],
                    z=X[2],
                ),
                orientation=Quaternion(
                    x=orient[0],
                    y=orient[1],
                    z=orient[2],
                    w=orient[3],
                ),
            ),
        ),
    }
				
    ikreq.pose_stamp.append(poses[limb])
    try:
        rospy.wait_for_service(ns, 5.0)
        resp = iksvc(ikreq)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("Service call failed: %s" % (e,))
        return 1

    # Check if result valid, and type of seed ultimately used to get solution
    # convert rospy's string representation of uint8[]'s to int's
    resp_seeds = struct.unpack('<%dB' % len(resp.result_type),
                               resp.result_type)
    if (resp_seeds[0] != resp.RESULT_INVALID):
        seed_str = {
                    ikreq.SEED_USER: 'User Provided Seed',
                    ikreq.SEED_CURRENT: 'Current Joint Angles',
                    ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                   }.get(resp_seeds[0], 'None')

        # Format solution into Limb API-compatible dictionary
        limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))


   	# create an instance of baxter_interface's Limb class
    	arm = baxter_interface.Limb(limb)

    	# reassign new joint angles (all zeros) which we will later command to the limb
    	if limb == 'right':
    	    # get the right limb's current joint angles
    	    angles = rarm.joint_angles() 
            angles['right_s0']= resp.joints[0].position[0]
    	    angles['right_s1']= resp.joints[0].position[1]
    	    angles['right_e0']= resp.joints[0].position[2]
    	    angles['right_e1']= resp.joints[0].position[3]
    	    angles['right_w0']= resp.joints[0].position[4]
    	    angles['right_w1']= resp.joints[0].position[5]
    	    angles['right_w2']= resp.joints[0].position[6]

        elif limb == 'left':
            # get the left limb's current joint angles
    	    angles = larm.joint_angles()
            angles['left_s0']= resp.joints[0].position[0]
    	    angles['left_s1']= resp.joints[0].position[1]
    	    angles['left_e0']= resp.joints[0].position[2]
    	    angles['left_e1']= resp.joints[0].position[3]
    	    angles['left_w0']= resp.joints[0].position[4]
    	    angles['left_w1']= resp.joints[0].position[5]
    	    angles['left_w2']= resp.joints[0].position[6]
    	
        # Return the joint angles
    	return angles

    # If no solution found give the message and return 0
    else:
        print("INVALID POSE - No Valid Joint Solution Found.")

    return 0

# Moves Baxter's arm to a default position
def defPos(limb):
    if limb == 'right':
        angles = rarm.joint_angles()
        angles['right_s0']=-1.6912138186436687
        angles['right_s1']=-0.01572330307582549
        angles['right_e0']= 1.5823011827038043
        angles['right_e1']=2.6077673394052034
        angles['right_w0']=-1.6037769137342002
        angles['right_w1']=-1.572330307582549
        angles['right_w2']=3.0572237102556294
        
        rarm.move_to_joint_positions(angles)

    if limb == 'left':
        angles = larm.joint_angles()
        angles['left_s0']=1.7019516841588667
        angles['left_s1']=-0.002684466378799474
        angles['left_e0']=-1.5999419617644866
        angles['left_e1']= 2.609301320193089
        angles['left_w0']=1.6175827408251688
        angles['left_w1']=-1.5704128315976924
        angles['left_w2']=3.0572237102556294

        larm.move_to_joint_positions(angles)

    return

#------------------------------------------------------------------------------
#                              mycalibrate()
#
# To find the relationship between VICON frame of reference's coordinates and
# Baxter's frame of reference, a calibration procedure is made. This
# mycalibrate moves Baxter's arm to a known position in Baxter's coordinates.
# Then, the user should place the object in Baxter's gripper and press enter.
# The function will get the coordinates of the object on VICON frame of
# reference. Now there is the same point calculated for both baxter an VICON.
# By subtracting the Baxter's coordinates to the Vicon coordinates for the
# same object, a transformation for VICON frame of reference to Baxter's is
# obtained. With this relationship (rel) any vicon coordinates can be
# transformed to Baxter's coordinates only by subtracting the last two.
#
# output
#     rel    The subtracting list between Baxter and VICON coordinates.
#
#------------------------------------------------------------------------------
def mycalibrate():

    #Baxter's Arm Coordinates for calibration.
    bac=[0.8,-0., 0.2]

    #Get joint angles
    angles = get_angles('right',bac)

    #Move to position bac.
    rarm.move_to_joint_positions(angles)

    print 'To calibrate, place the object in Baxters right gripper.'
    
    # Wait for the object to be placed on Baxter's gripper. 
    raw_input("Press Enter when ready...")
    retrieveSponge()
    
    # VICON's Object Coordinates on Baxter's arm for calibration. VICON
    # x and y cordinates are inverse to Baxter's coordinates.
    
    voc = [-spongeCor[0], -spongeCor[1],spongeCor[2] ]

    # rel is a list that has to be subtracted to the VICON coordinates to
    # transform from Vicon coordinates to Baxter coordinates.
    r = [a - b for a, b in zip(voc, bac)]

    return r

#------------------------------------------------------------------------------
#                            grab(limb,r)
# 
# Moves Baxter's arm to 0.1 m. from the object. Then, it reaches it slowly 2 cm
# ant the time and closes the gripper. Calibration is needed before executing
# this function.
#
# inputs
#     r       The list obtained by mycalibrate(). It is used to transform form
#             VICON coordinates to Baxter's Coordinates
# 
#------------------------------------------------------------------------------
def grab(r):
    
    # Find the VICON coordinates of the object (sponge)
    retrieveSponge()
    voc = [-spongeCor[0], -spongeCor[1], spongeCor[2]]

    # Transform to Baxter coordinates using the calibration list. And get points
    # to approach the sponge slowly by steps each 2cm.
    boc6 = [a - b for a, b in zip(voc, r)]
    boc1 = [boc6[0]-0.1,boc6[1],boc6[2]]
    boc2 = [boc6[0]-0.08,boc6[1],boc6[2]]
    boc3 = [boc6[0]-0.06,boc6[1],boc6[2]]
    boc4 = [boc6[0]-0.04,boc6[1],boc6[2]]
    boc5 = [boc6[0]-0.02,boc6[1],boc6[2]]
    
    # if the object is on the left side of Baxter use the left arm. If it is on
    # the right side use the right limb.

    if boc6[1] > 0.01:
        limb = 'left'
    else:
        limb = 'right'

    #Get joint angles for all the pints
    angles1 = get_angles(limb,boc1)
    angles2 = get_angles(limb,boc2)
    angles3 = get_angles(limb,boc3)
    angles4 = get_angles(limb,boc4)
    angles5 = get_angles(limb,boc5)
    angles6 = get_angles(limb,boc6)
    
    # If there is an invalid pose, get_angles returns 0. If there was any
    # invalid pose, exit the function. 
    if angles1==0 or angles2 == 0 or angles3 == 0 or angles4 == 0 or angles5 == 0 or angles6 == 0:
        return 'NO'

    # Make the sequence of movement to approach the object. And then, close
    # gripper.
    if limb =='right':
        defPos('left')
        rarm.move_to_joint_positions(angles1)
        rarm.move_to_joint_positions(angles2)
        rarm.move_to_joint_positions(angles3)
        rarm.move_to_joint_positions(angles4)
        rarm.move_to_joint_positions(angles5)
        rarm.move_to_joint_positions(angles6)
        #close gripper
        rgripper.close()
    
    elif limb =='left':
        defPos('right')
        larm.move_to_joint_positions(angles1)
        larm.move_to_joint_positions(angles2)
        larm.move_to_joint_positions(angles3)
        larm.move_to_joint_positions(angles4)
        larm.move_to_joint_positions(angles5)
        larm.move_to_joint_positions(angles6)
        #close gripper
        lgripper.close()

    return limb

#------------------------------------------------------------------------------
#                           toBin(limb, r)
#
# It moves the gripper to 25 cm above the bin, opens the gripper and returns to
# a default position.
#
# input
#     r    The list obtained by mycalibrate(). It is used to transform form
#          VICON coordinates to Baxter's Coordinates
#    
#------------------------------------------------------------------------------
def toBin(limb, r):
  
    # Find the VICON coordinates of the bin and shift 25cm over. Then, transform
    # them to Baxter coordinates using 'rel' obtained in calibration.
    getBin()

    vbc=[-binCor[0], -binCor[1], binCor[2]+0.25] 
    bbc = [a - b for a, b in zip(vbc, r)]

    #Get joint angles
    angles = get_angles(limb,bbc)
    if angles==0:
        print 'Cannot reach the bin'
        lgripper.open()
        rgripper.open() 
        defPos('left')
        defPos('right')
        return 

    #Move to position bbc and open gripper.
    if limb =='right':
        rarm.move_to_joint_positions(angles)
        rgripper.open()
    else: 
        larm.move_to_joint_positions(angles)
        lgripper.open()

    # Return the arm to a default position.
    defPos(limb)
    return 

if __name__=='__main__':
    retrieveSponge()
    getBin()
    rel=[]
    defPos('left')
    print "Starting calibration..."
    rel = mycalibrate()
    a = 'c'
    while a != 'e':
        a = raw_input('To pick the object entre "o", to calibrate enter "c" and to exti enter "e" ')
        if a == 'o':
            
            # Approaches the object and grabs it.   
            print "Searching for object..." 
            arm = grab(rel)
            
            if arm != 'NO':               
                # Moves to the bin and releases the object.
                print "Moving to bin..."
                toBin(arm, rel)
            else:
                print 'Sorry, I cannot reach the object.'
                      
        elif a == 'c':
            print "Starting calibration..."
            rel = mycalibrate()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "shutting down"


