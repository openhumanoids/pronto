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
from bot_core.ins_t import ins_t
from bot_core.images_t import images_t
#from pronto.atlas_raw_imu_batch_t import atlas_raw_imu_batch_t
#from pronto.atlas_state_t import atlas_state_t
########################################################################################

def timestamp_now (): return int (time.time () * 1000000)

# mode 0 - visual
# mode 1 - for matlab
global mode
mode = 0


def on_pose(channel, data):

  m = pose_t.decode(data)
  if (mode is 0):
    print m.utime*1E-6, " vic"
  else:
    print m.utime*1E-6,",0,0"

def on_ins(channel, data):
  m = ins_t.decode(data)
  if (mode is 0):
    print "                        ", m.utime*1E-6, " ins"
  else:
    print "0,", m.utime*1E-6, ",0"


def on_imu(channel, data):
  m = atlas_raw_imu_batch_t.decode(data)
  if (mode is 0):
    print "                        ", m.utime*1E-6, " imu"
  else:
    print "0,", m.utime*1E-6, ",0"

def on_atlas_state(channel, data):
  m = atlas_state_t.decode(data)
  if (mode is 0):
    print "                                                  ",m.utime*1E-6, " jnt"
  else:
    print "0,0,",m.utime*1E-6


def on_camera(channel, data):
  m = images_t.decode(data)
  if (mode is 0):
    print "                                                  ",m.utime*1E-6, " cam"
  else:
    print "0,0,",m.utime*1E-6

#################################################################################

lc = lcm.LCM()
print "started"

lc.subscribe("POSE_GROUND_TRUTH", on_pose)
lc.subscribe("MICROSTRAIN_INS", on_ins)

lc.subscribe("ATLAS_IMU_BATCH", on_imu)
lc.subscribe("ATLAS_STATE", on_atlas_state)

lc.subscribe("CAMERA", on_camera)
  
while True:
  ## Handle LCM if new messages have arrived.
  lc.handle()

