
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

home_dir =os.getenv("DRC_BASE")
#print home_dir
sys.path.append(home_dir + "/software/build/lib/python2.7/site-packages")
sys.path.append(home_dir + "/software/build/lib/python2.7/dist-packages")

from bot_core.pose_t import pose_t
from pronto.indexed_measurement_t import indexed_measurement_t

def timestamp_now (): return int (time.time () * 1000000)

def on_pose_body(channel, data):
  m = pose_t.decode(data)

  out = indexed_measurement_t()

  out.utime = m.utime;
  out.state_utime = m.utime;

  out.measured_dim = 4;
  out.z_indices = [8,9,10,11] # yaw, x, y, z
  #msg->state[8]
  # out.z_effective = { rpyl[2] , msg->state[9], msg->state[10], msg->state[11]};
  out.z_effective = [0.1,0,0,0]# msg->state[8] , msg->state[9], msg->state[10], msg->state[11]};
  out.measured_cov_dim = 16;
  out.R_effective = [0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0]
  out.R_effective[0]  = 1000#cl_cfg_.yaw_cov; // yaw
  out.R_effective[5]  = 1000#cl_cfg_.xyz_cov; // x
  out.R_effective[10] = 1000#cl_cfg_.xyz_cov; // y
  out.R_effective[15] = 1000#cl_cfg_.xyz_cov; // z

  lc.publish("GPF_MEASUREMENT_QUICK_LOCK", out.encode())



####################################################################
lc = lcm.LCM()
print "started"

sub1 = lc.subscribe("POSE_BODY", on_pose_body)

while True:
  ## Handle LCM if new messages have arrived.
  lc.handle()

lc.unsubscribe(sub1)



