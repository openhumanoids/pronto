#!/usr/bin/python
# Republish old ATLAS_STATE message (from logs)
# As CORE_ROBOT_STATE and FORCE_TORQUE 
 

import os,sys
import lcm
import time
from lcm import LCM
from math import *
import numpy  as np
import matplotlib.pyplot as plt
import matplotlib.mlab as mlab

from threading import Thread
import threading

home_dir =os.getenv("DRC_BASE")
#print home_dir
sys.path.append(home_dir + "/software/build/lib/python2.7/site-packages")
sys.path.append(home_dir + "/software/build/lib/python2.7/dist-packages")

from pronto.atlas_state_t import atlas_state_t
from pronto.joint_state_t import joint_state_t
from pronto.force_torque_t import force_torque_t
from pronto.six_axis_force_torque_array_t import six_axis_force_torque_array_t
from pronto.six_axis_force_torque_t import six_axis_force_torque_t

atlas_version = 5

# atlas :
if atlas_version == 3:
    atlas_joint_names = ["back_bkz", "back_bky", "back_bkx", 
        "neck_ay", "l_leg_hpz", "l_leg_hpx", "l_leg_hpy", 
        "l_leg_kny", "l_leg_aky", "l_leg_akx", "r_leg_hpz", 
        "r_leg_hpx", "r_leg_hpy", "r_leg_kny", "r_leg_aky", 
        "r_leg_akx", "l_arm_usy", "l_arm_shx", "l_arm_ely", 
        "l_arm_elx", "l_arm_uwy", "l_arm_mwx", "r_arm_usy", 
        "r_arm_shx", "r_arm_ely", "r_arm_elx", "r_arm_uwy", "r_arm_mwx"]
elif atlas_version == 5:
    atlas_joint_names = ["back_bkz", "back_bky", "back_bkx", 
        "neck_ay", "l_leg_hpz", "l_leg_hpx", "l_leg_hpy", 
        "l_leg_kny", "l_leg_aky", "l_leg_akx", "r_leg_hpz", 
        "r_leg_hpx", "r_leg_hpy", "r_leg_kny", "r_leg_aky", 
        "r_leg_akx", "l_arm_shz", "l_arm_shx", "l_arm_ely", 
        "l_arm_elx", "l_arm_uwy", "l_arm_mwx", "l_arm_lwy", "r_arm_shz", 
        "r_arm_shx", "r_arm_ely", "r_arm_elx", "r_arm_uwy", "r_arm_mwx", "r_arm_lwy"]


########################################################################################
def timestamp_now (): return int (time.time () * 1000000)

def on_atlas_state(channel, data):
  m = atlas_state_t.decode(data)
  o = joint_state_t()
  o.utime = m.utime
  o.num_joints = m.num_joints
  o.joint_name = atlas_joint_names
  o.joint_position = m.joint_position
  o.joint_velocity = m.joint_velocity
  o.joint_effort = m.joint_effort
  lc.publish("CORE_ROBOT_STATE", o.encode()) 

  ft = m.force_torque
  lf = six_axis_force_torque_t()
  lf.utime = m.utime
  lf.force = [0, 0, ft.l_foot_force_z]
  lf.moment = [ft.l_foot_torque_x, ft.l_foot_torque_y, 0]
  rf = six_axis_force_torque_t()
  rf.utime = m.utime
  rf.force = [0, 0, ft.r_foot_force_z]
  rf.moment = [ft.r_foot_torque_x, ft.r_foot_torque_y, 0]
  lh = six_axis_force_torque_t()
  lh.utime = m.utime
  lh.force = ft.l_hand_force
  lh.moment = ft.l_hand_torque
  rh = six_axis_force_torque_t()
  rh.utime = m.utime
  rh.force = ft.r_hand_force
  rh.moment = ft.r_hand_torque

  msg_out = six_axis_force_torque_array_t()
  msg_out.utime = m.utime
  msg_out.sensors = [lf, rf, lh, rh]
  msg_out.names =['l_foot', 'r_foot', 'l_hand', 'r_hand']
  msg_out.num_sensors = len(msg_out.sensors)
  lc.publish("FORCE_TORQUE", msg_out.encode())
  

####################################################################
lc = lcm.LCM()
print "started"

sub1 = lc.subscribe("ATLAS_STATE", on_atlas_state)

while True:
  ## Handle LCM if new messages have arrived.
  lc.handle()

lc.unsubscribe(sub1)



