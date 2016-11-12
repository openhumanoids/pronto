#!/usr/bin/python

import os,sys
import lcm
import time
from lcm import LCM
import math

from bot_core.pose_t import pose_t
import botpy
########################################################################################

def timestamp_now (): return int (time.time () * 1000000)

print "#CHANNEL,utime,pos_x,pos_y,pos_z,quat_w,quat_x,quat_y,quat_z,roll,pitch,yaw"

global str_alt, str_body, str_vicon
global utime_last_print
utime_last_print = 0
str_alt =None
str_body =None
str_vicon =None

def on_pose(channel, data):
  global utime_last_print, str_alt, str_body, str_vicon
  m = pose_t.decode(data)
  rpy = botpy.quat_to_euler(m.orientation)


  str_out = channel + ", " + str(m.utime) + ", "
  str_out = str_out + str(m.pos[0]) + ", " + str(m.pos[1]) + ", " + str(m.pos[2]) + ", "
  str_out = str_out + str(m.orientation[0]) + ", " + str(m.orientation[1]) + ", " + str(m.orientation[2]) + ", " + str(m.orientation[3]) + ", "
  str_out = str_out + str(rpy[0]*180.0/math.pi) + ", " + str(rpy[1]*180.0/math.pi) + ", " + str(rpy[2]*180.0/math.pi)

  if (channel == "POSE_BODY_ALT"):
    str_alt = str_out
  elif (channel == "POSE_BODY"):
    str_pose = str_out
  elif (channel == "POSE_VICON"):
    str_vicon = str_out

  if (m.utime > utime_last_print + 1E6):
    if (channel == "POSE_BODY"):
      if (str_alt is None):
        return
      if (str_pose is None):
        return
      if (str_vicon is None):
        return

      utime_last_print = m.utime
      print str_pose
      print str_alt
      print str_vicon

#################################################################################

lc = lcm.LCM()

lc.subscribe("POSE_BODY_ALT", on_pose)
lc.subscribe("POSE_BODY", on_pose)
lc.subscribe("POSE_VICON", on_pose)
  
while True:
  ## Handle LCM if new messages have arrived.
  lc.handle()

