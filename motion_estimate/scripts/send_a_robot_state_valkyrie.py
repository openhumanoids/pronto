#!/usr/bin/python
# A very simple process to combine the floating base estimate
# with the kinematics and output the combined message
# input: POSE_BODY and ATLAS_STATE, output: EST_ROBOT_STATE

import os,sys
import lcm
import time
from lcm import LCM
from math import *
import numpy  as np
import numpy.random as random

home_dir =os.getenv("HOME")
sys.path.append(home_dir + "/drc/software/build/lib/python2.7/site-packages")
sys.path.append(home_dir + "/drc/software/build/lib/python2.7/dist-packages")

from bot_core.robot_state_t import robot_state_t
########################################################################################
def timestamp_now (): return int (time.time () * 1000000)


joint_name_list = ['torsoYaw','torsoPitch','torsoRoll','lowerNeckPitch','neckYaw','upperNeckPitch','rightShoulderPitch','rightShoulderRoll','rightShoulderYaw','rightElbowPitch','rightForearmYaw','rightWristRoll','rightWristPitch','leftShoulderPitch','leftShoulderRoll','leftShoulderYaw','leftElbowPitch','leftForearmYaw','leftWristRoll','leftWristPitch','rightHipYaw','rightHipRoll','rightHipPitch','rightKneePitch','rightAnklePitch','rightAnkleRoll','leftHipYaw','leftHipRoll','leftHipPitch','leftKneePitch','leftAnklePitch','leftAnkleRoll']

def send_state():
  o = robot_state_t()
  o.utime = timestamp_now ()
  o.num_joints = len(joint_name_list)
  o.joint_name = joint_name_list
  o.joint_position = random.rand(o.num_joints) -0.25


  o.joint_velocity = [0]*o.num_joints
  o.joint_effort = [0]*o.num_joints

  o.pose.translation.x =0;
  o.pose.translation.y =0;
  o.pose.translation.z =1;
  o.pose.rotation.w = 1;
  o.pose.rotation.x = 0
  o.pose.rotation.y = 0
  o.pose.rotation.z = 0

  lc.publish("EST_ROBOT_STATE_ALT",o.encode())  

####################################################################
lc = lcm.LCM()
print "started"

send_state()