#!/usr/bin/python

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

from drc.atlas_state_t import atlas_state_t

########################################################################################
def timestamp_now (): return int (time.time () * 1000000)

class State:
  def __init__(self):
    self.l_foot_force_z_prev = None
    self.r_foot_force_z_prev = None

def on_atlas_state(channel, data):
  global state
  m = atlas_state_t.decode(data)
  alpha = 0.05

  if (state.l_foot_force_z_prev is None):
      # do nothing
      m.force_torque.l_foot_force_z = m.force_torque.l_foot_force_z
      m.force_torque.r_foot_force_z = m.force_torque.r_foot_force_z
  else:
      m.force_torque.l_foot_force_z = alpha*m.force_torque.l_foot_force_z + (1-alpha)*state.l_foot_force_z_prev
      m.force_torque.r_foot_force_z = alpha*m.force_torque.r_foot_force_z + (1-alpha)*state.r_foot_force_z_prev

  lc.publish("ATLAS_STATE", m.encode() )
  state.l_foot_force_z_prev = m.force_torque.l_foot_force_z
  state.r_foot_force_z_prev = m.force_torque.r_foot_force_z

lc = lcm.LCM()
print "started"
state = State()
sub1 = lc.subscribe("ATLAS_STATE_AS_LOGGED", on_atlas_state)
while True:
  lc.handle()
lc.unsubscribe(sub1)



