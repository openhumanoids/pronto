#!/usr/bin/python

import os,sys
import lcm
import time
from lcm import LCM
import math

from bot_core.pose_t import pose_t
from pronto.error_metrics_t import error_metrics_t
import botpy
from botpy import BotTrans
import numpy as np
########################################################################################

def timestamp_now (): return int (time.time () * 1000000)

class State(object):
    def __init__(self):
        """Return a Customer object whose name is *name* and starting
        balance is *balance*."""
        # Ground truth:
        self.last_est = pose_t()
        self.last_est.utime = -2
        self.counter =0

s =State()
def poseToBotTrans(pose):
  btOut = BotTrans()
  btOut.assign( np.array(pose.pos), np.array(pose.orientation) )
  return btOut


def on_pose_est(channel, data):
  m = pose_t.decode(data)  
  s.last_est = m


def on_pose_gt(channel, data):
  m = pose_t.decode(data)

  rpy = botpy.quat_to_euler(m.orientation)

  # pre-init:
  if (s.last_est.utime < 0):
    return


  # after the reliable time
  s.counter = s.counter + 1
  if (s.counter % 10 == 1):
    string1 = ''
    string1 = string1 + str(m.utime) +", "+ str(m.pos[0]) +", "+ str(m.pos[1]) +", "+ str(m.pos[2])
    string1 = string1+ ", " +str(m.orientation[0]) +", "+ str(m.orientation[1]) +", "+ str(m.orientation[2]) +", "+ str(m.orientation[3])

    d= s.last_est
    string1 = string1 + ", " +str(d.utime) +", "+ str(d.pos[0]) +", "+ str(d.pos[1]) +", "+ str(d.pos[2])
    string1 = string1 + ", " +str(d.orientation[0]) +", "+ str(d.orientation[1]) +", "+ str(m.orientation[2]) +", "+ str(d.orientation[3])

    print string1



#################################################################################

lc = lcm.LCM()

lc.subscribe("POSE_BODY", on_pose_est)
lc.subscribe("POSE_GROUND_TRUTH", on_pose_gt)
  
while True:
  ## Handle LCM if new messages have arrived.
  lc.handle()

