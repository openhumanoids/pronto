#!/usr/bin/python
# A very simple process to combine the floating base estimate
# with the kinematics and output the combined message
# input: POSE_BODY and ATLAS_STATE, output: EST_ROBOT_STATE
#
# currently this only works/used for Thor Mang

import os,sys
import lcm
import time
from lcm import LCM
from math import *
import numpy  as np
import matplotlib.pyplot as plt
import matplotlib.mlab as mlab

home_dir =os.getenv("HOME")
#print home_dir
sys.path.append(home_dir + "/drc/software/build/lib/python2.7/site-packages")
sys.path.append(home_dir + "/drc/software/build/lib/python2.7/dist-packages")
#sys.path.append(home_dir + "/otherprojects/pronto-distro/build/lib/python2.7/site-packages")
#sys.path.append(home_dir + "/otherprojects/pronto-distro/build/lib/python2.7/dist-packages")

from bot_core.pose_t import pose_t
from pronto.robot_state_t import robot_state_t
from pronto.atlas_state_t import atlas_state_t
########################################################################################
def timestamp_now (): return int (time.time () * 1000000)

global atlas_state

atlas_state = atlas_state_t()

# thor mang:
joint_name_list = [u'head_pan', u'head_tilt', u'l_ankle_pitch', u'l_ankle_roll', u'l_elbow', u'l_hip_pitch', u'l_hip_roll', u'l_hip_yaw', u'l_knee', u'l_shoulder_pitch', u'l_shoulder_roll', u'l_shoulder_yaw', u'l_wrist_roll', u'l_wrist_yaw1', u'l_wrist_yaw2', u'r_ankle_pitch', u'r_ankle_roll', u'r_elbow', u'r_hip_pitch', u'r_hip_roll', u'r_hip_yaw', u'r_knee', u'r_shoulder_pitch', u'r_shoulder_roll', u'r_shoulder_yaw', u'r_wrist_roll', u'r_wrist_yaw1', u'r_wrist_yaw2', u'waist_lidar', u'waist_pan', u'waist_tilt']


def on_atlas_state(channel, data):
  global atlas_state
  atlas_state = atlas_state_t.decode(data)

def on_pose_body(channel, data):
  global atlas_state, joint_names
  if (atlas_state.num_joints==0):
    return

  m = pose_t.decode(data)

  o = robot_state_t()
  o.utime = m.utime
  o.num_joints = atlas_state.num_joints
  #o.joint_name = ["" for x in range(o.num_joints)]
  o.joint_name = joint_name_list
  o.joint_position = atlas_state.joint_position
  o.joint_velocity = atlas_state.joint_velocity
  o.joint_effort = atlas_state.joint_effort


  o.pose.translation.x =m.pos[0];
  o.pose.translation.y =m.pos[1];
  o.pose.translation.z =m.pos[2];
  o.pose.rotation.w = m.orientation[0];
  o.pose.rotation.x = m.orientation[1];
  o.pose.rotation.y = m.orientation[2];
  o.pose.rotation.z = m.orientation[3];

  lc.publish("EST_ROBOT_STATE",o.encode())  

####################################################################
lc = lcm.LCM()
print "started"


sub1 = lc.subscribe("ATLAS_STATE", on_atlas_state)
sub2 = lc.subscribe("POSE_BODY", on_pose_body)

while True:
  lc.handle()
