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

from pronto.multisense_state_t import multisense_state_t

########################################################################################
def timestamp_now (): return int (time.time () * 1000000)

def on_multisense_state(channel, data):
  m = multisense_state_t.decode(data)
  m.joint_name[0] = 'hokuyo_joint'
  lc.publish("MULTISENSE_STATE", m.encode()) 


####################################################################
lc = lcm.LCM()
print "started"

sub1 = lc.subscribe("MULTISENSE_STATE_LOGGED", on_multisense_state)

while True:
  ## Handle LCM if new messages have arrived.
  lc.handle()

lc.unsubscribe(sub1)



