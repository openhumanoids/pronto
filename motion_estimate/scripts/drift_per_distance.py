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

#global str_alt, str_body, str_vicon
#global utime_last_print
#utime_last_print = 0
#str_alt =None
#str_body =None
#str_vicon =None

class State(object):
    def __init__(self):
        """Return a Customer object whose name is *name* and starting
        balance is *balance*."""
        # Ground truth:
        self.last = pose_t()
        self.last.utime = -2

        self.last_est = pose_t()
        self.last_est.utime = -2

        self.most_recent_est = pose_t()
        self.most_recent_est.utime = -2

        self.parameterDDTThreshold = 0.25
        self.parameterTimeElapsedThreshold = 10.0

    def withdraw(self):
        return self.balance

s = State()

def poseToBotTrans(pose):
  btOut = BotTrans()
  btOut.assign( np.array(pose.pos), np.array(pose.orientation) )
  return btOut


def on_pose_est(channel, data):
  m = pose_t.decode(data)
  #print '%.6f' % (m.utime*1e-6)  
  s.most_recent_est = m


def decide_new_measurement(m):
  distanceTravelled = np.linalg.norm(np.array(m.pos) - np.array(s.last.pos))
  #print distanceTravelled
  #if (distanceTravelled> s.parameterDDTThreshold):
  #  return True

  if (m.utime - s.last.utime > s.parameterTimeElapsedThreshold*1e6):
    return True

  return False

def on_pose_gt(channel, data):
  m = pose_t.decode(data)
  #print '                            %.6f' % (m.utime*1e-6)  

  rpy = botpy.quat_to_euler(m.orientation)

  # pre-init:
  if (s.most_recent_est.utime < 0):
    return
  if (s.last.utime < 0):
    s.last = m
    s.last_est = s.most_recent_est
    return

  # after the reliable time
  #if (m.utime - s.last.utime > 1e6):
  if decide_new_measurement(m):
    #print '%.6f' % (s.last.utime*1e-6)

    gt_a = poseToBotTrans(s.last)
    gt_b = poseToBotTrans(m)
    gt_ab = botpy.transform_relative(gt_a, gt_b)

    se_a = poseToBotTrans(s.last_est)
    se_b = poseToBotTrans(s.most_recent_est)
    se_ab = botpy.transform_relative(se_a, se_b)

    #print "==========="
    #print gt_ab.trans_vec
    #print se_ab.trans_vec

    distanceTravelled = np.linalg.norm(np.array(m.pos) - np.array(s.last.pos))
    drift_xyz = np.array(se_ab.trans_vec) - np.array(gt_ab.trans_vec)
    drift = np.linalg.norm(np.array(se_ab.trans_vec) - np.array(gt_ab.trans_vec))
    percentDriftPerDistanceTravelled = 100*drift/distanceTravelled

    # yaw is postive anti clockwise, zero facing towards x
    gt_ab_yaw = botpy.quat_to_euler(gt_ab.rot_quat)[2]*180.0/np.pi
    se_ab_yaw = botpy.quat_to_euler(se_ab.rot_quat)[2]*180.0/np.pi
    yaw_error = se_ab_yaw - gt_ab_yaw

    #print "======================"
    #print "======================"
    #print "yaw gt pre: ", botpy.quat_to_euler(s.last.orientation)[2]*180.0/np.pi
    #print "yaw gt now: ", botpy.quat_to_euler(m.orientation)[2]*180.0/np.pi
    #print "yaw se pre: ", botpy.quat_to_euler(s.last_est.orientation)[2]*180.0/np.pi
    #print "yaw se now: ", botpy.quat_to_euler(s.most_recent_est.orientation)[2]*180.0/np.pi
    #print "======================"
    #print gt_ab_yaw
    #print se_ab_yaw

    print yaw_error, "deg yaw drift |", drift_xyz, "or" , drift, "m drift in", distanceTravelled , "m travelled, thus" , percentDriftPerDistanceTravelled, "%DDT" 


    em = error_metrics_t()
    em.utime = m.utime
    em.pos_error = drift_xyz
    em.pos_error_norm = drift
    em.rpy_error = [0,0,yaw_error]

    em.distance_travelled = distanceTravelled
    em.time_elapsed = (s.last.utime - m.utime)*1e-6
    em.percent_ddt = percentDriftPerDistanceTravelled
    lc.publish("PRONTO_ERROR", em.encode())


    s.last = m
    s.last_est = s.most_recent_est
    return



#################################################################################

lc = lcm.LCM()

lc.subscribe("POSE_BODY", on_pose_est)
lc.subscribe("POSE_GROUND_TRUTH", on_pose_gt)
  
while True:
  ## Handle LCM if new messages have arrived.
  lc.handle()

