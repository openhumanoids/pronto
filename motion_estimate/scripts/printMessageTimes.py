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
from bot_core.pose_t import pose_t
from drc.atlas_raw_imu_batch_t import atlas_raw_imu_batch_t
from drc.atlas_state_t import atlas_state_t
########################################################################################

def timestamp_now (): return int (time.time () * 1000000)
 
def on_pose(channel, data):
  m = pose_t.decode(data)
  print m.utime, " bdi"

def on_imu(channel, data):
  m = atlas_raw_imu_batch_t.decode(data)
  print "                        ", m.utime, " imu"

def on_atlas_state(channel, data):
  m = atlas_state_t.decode(data)
  print "                                                  ",m.utime, " jnt"

#################################################################################

lc = lcm.LCM()
print "started"

lc.subscribe("POSE_BDI", on_pose)
lc.subscribe("ATLAS_IMU_BATCH", on_imu)
lc.subscribe("ATLAS_STATE", on_atlas_state)
  
while True:
  ## Handle LCM if new messages have arrived.
  lc.handle()

