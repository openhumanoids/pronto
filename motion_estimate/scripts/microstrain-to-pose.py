#!/usr/bin/python
# republish microstrain as pose (orientation only)

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

from mav.ins_t import ins_t
from bot_core.pose_t import pose_t
from pronto.robot_state_t import robot_state_t
from pronto.joint_state_t import joint_state_t
########################################################################################
def timestamp_now (): return int (time.time () * 1000000)

global joint_state






def on_ms(channel, data):
  m = ins_t.decode(data)
  print m.utime

  o = pose_t()
  o.utime = m.utime
  o.pos = [0,0,1]
  o.orientation = m.quat
  lc.publish("POSE_BODY_ALT",o.encode())  

def on_pose_body(channel, data):
  print "pose"
  #lc.publish("EST_ROBOT_STATE",o.encode())  

####################################################################
lc = lcm.LCM()
print "started"


sub1 = lc.subscribe("MICROSTRAIN_INS", on_ms)
#sub2 = lc.subscribe("POSE_BODY", on_pose_body)

while True:
  lc.handle()
