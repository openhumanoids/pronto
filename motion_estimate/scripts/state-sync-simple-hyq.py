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
from bot_core.robot_state_t import robot_state_t
from bot_core.joint_state_t import joint_state_t
########################################################################################
def timestamp_now (): return int (time.time () * 1000000)

global joint_state

joint_state = joint_state_t()

joint_state.joint_name = ['lf_haa_joint', 'lf_hfe_joint', 'lf_kfe_joint', 'rf_haa_joint', 'rf_hfe_joint', 'rf_kfe_joint', 'lh_haa_joint', 'lh_hfe_joint', 'lh_kfe_joint', 'rh_haa_joint', 'rh_hfe_joint', 'rh_kfe_joint', 'ptu_pan', 'ptu_tilt']
joint_state.num_joints = len(joint_state.joint_name)
joint_state.joint_position = [0] * len(joint_state.joint_name)
joint_state.joint_velocity = [0] * len(joint_state.joint_name)
joint_state.joint_effort = [0] * len(joint_state.joint_name)
received_joint_state = False

skip_joint_angles = True
if skip_joint_angles:
  received_joint_state = True


def on_joint_state(channel, data):
  global joint_state, received_joint_state
  joint_state = joint_state_t.decode(data)
  received_joint_state = True


def on_pose_body(channel, data):
  if (skip_joint_angles is False):
    if (received_joint_state is False):
      return

  m = pose_t.decode(data)

  o = robot_state_t()
  o.utime = m.utime

  o.num_joints = joint_state.num_joints
  o.joint_name = joint_state.joint_name
  o.joint_position = joint_state.joint_position
  o.joint_velocity = joint_state.joint_velocity
  o.joint_effort = joint_state.joint_effort

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


sub1 = lc.subscribe("HYQ_STATE", on_joint_state)
sub2 = lc.subscribe("POSE_BODY", on_pose_body)

while True:
  lc.handle()
