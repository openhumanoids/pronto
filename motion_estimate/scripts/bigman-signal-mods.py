#!/usr/bin/python
# MIT uses hokuyo_joint
# other teams use motor_joint, rename here

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

home_dir =os.getenv("HOME")
#print home_dir
sys.path.append(home_dir + "/drc/software/build/lib/python2.7/site-packages")
sys.path.append(home_dir + "/drc/software/build/lib/python2.7/dist-packages")

from pronto.six_axis_force_torque_array_t import six_axis_force_torque_array_t
from mav.ins_t import ins_t
from pronto.joint_state_t import joint_state_t

########################################################################################
def timestamp_now (): return int (time.time () * 1000000)

def on_crs(channel, data):
  m = joint_state_t.decode(data)
  print m.utime

def on_ft(channel, data):
  m = six_axis_force_torque_array_t.decode(data)

  o = m
  s_r = o.sensors[0]
  s_l = o.sensors[1]
  n_r = o.names[0]
  n_l = o.names[1]

  o.sensors[0] = s_l
  o.sensors[1] = s_r
  o.names[0] = n_l
  o.names[1] = n_r

  lc.publish("FORCE_TORQUE", o.encode()) 

def on_ms(channel, data):
  m = ins_t.decode(data)
  o = m
  #o.accel = [0,0,-9.81]
  o.gyro = [m.gyro[0]/100.0, m.gyro[1]/100.0, m.gyro[2]/100.0]
  lc.publish("MICROSTRAIN_INS", o.encode()) 


####################################################################
lc = lcm.LCM()
print "started"

sub1 = lc.subscribe("FORCE_TORQUE_LOGGED", on_ft)

sub2 = lc.subscribe("MICROSTRAIN_INS_LOGGED", on_ms)

#sub3 = lc.subscribe("CORE_ROBOT_STATE", on_crs)

while True:
  ## Handle LCM if new messages have arrived.
  lc.handle()

lc.unsubscribe(sub1)



