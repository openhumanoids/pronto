#!/usr/bin/python

import os,sys
import lcm
import time
from lcm import LCM
import math

home_dir =os.getenv("HOME")
#print home_dir
sys.path.append(home_dir + "/drc/software/build/lib/python2.7/site-packages")
sys.path.append(home_dir + "/drc/software/build/lib/python2.7/dist-packages")
sys.path.append(home_dir + "/otherprojects/pronto-distro/build/lib/python2.7/site-packages")
sys.path.append(home_dir + "/otherprojects/pronto-distro/build/lib/python2.7/dist-packages")

from bot_core.pose_t import pose_t
from pronto.atlas_raw_imu_batch_t import atlas_raw_imu_batch_t
from pronto.atlas_state_t import atlas_state_t
########################################################################################

def timestamp_now (): return int (time.time () * 1000000)

# mode 0 - visual
# mode 1 - for matlab
mode = 1
global mode

def on_pose(channel, data):

  m = pose_t.decode(data)
  if (mode is 0):
    print m.utime, " bdi"
  else:
    print m.utime,",0,0"

def on_imu(channel, data):
  m = atlas_raw_imu_batch_t.decode(data)
  if (mode is 0):
    print "                        ", m.utime, " imu"
  else:
    print "0,", m.utime, ",0"

def on_atlas_state(channel, data):
  m = atlas_state_t.decode(data)
  if (mode is 0):
    print "                                                  ",m.utime, " jnt"
  else:
    print "0,0,",m.utime

#################################################################################

lc = lcm.LCM()
print "started"

lc.subscribe("POSE_BDI", on_pose)
lc.subscribe("ATLAS_IMU_BATCH", on_imu)
lc.subscribe("ATLAS_STATE", on_atlas_state)
  
while True:
  ## Handle LCM if new messages have arrived.
  lc.handle()

