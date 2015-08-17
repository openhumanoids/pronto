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
from pronto.vector_3d_t import vector_3d_t
from pronto.position_3d_t import position_3d_t
from pronto.twist_t import twist_t
from pronto.quaternion_t import quaternion_t
from pronto.twist_t import twist_t
from pronto.force_torque_t import force_torque_t

########################################################################################
def timestamp_now (): return int (time.time () * 1000000)

global atlas_state

atlas_state = atlas_state_t()

# atlas :
joint_name_list = ["back_bkz", "back_bky", "back_bkx", 
        "neck_ay", "l_leg_hpz", "l_leg_hpx", "l_leg_hpy", 
        "l_leg_kny", "l_leg_aky", "l_leg_akx", "r_leg_hpz", 
        "r_leg_hpx", "r_leg_hpy", "r_leg_kny", "r_leg_aky", 
        "r_leg_akx", "l_arm_usy", "l_arm_shx", "l_arm_ely", 
        "l_arm_elx", "l_arm_uwy", "l_arm_mwx", "r_arm_usy", 
        "r_arm_shx", "r_arm_ely", "r_arm_elx", "r_arm_uwy", "r_arm_mwx"]


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


  nrot = quaternion_t()
  nvec = vector_3d_t()
  p = position_3d_t()
  p.rotation = nrot
  p.translation = nvec
  o.pose = p
 
  t = twist_t()
  t.linear_velocity = nvec
  t.angular_velocity = nvec
  o.twist = t

  ft = force_torque_t()
  o.force_torque = ft

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
