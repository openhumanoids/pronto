#!/usr/bin/python
import os,sys
import lcm
import time

home_dir =os.getenv("HOME")
sys.path.append(home_dir + "/drc/software/build/lib/python2.7/site-packages")
sys.path.append(home_dir + "/drc/software/build/lib/python2.7/dist-packages")
#sys.path.append(home_dir + "/otherprojects/pronto-distro/build/lib/python2.7/site-packages")
#sys.path.append(home_dir + "/otherprojects/pronto-distro/build/lib/python2.7/dist-packages")

from pronto.utime_t import utime_t
from mav.indexed_measurement_t import indexed_measurement_t
########################################################################################
def timestamp_now (): return int (time.time () * 1000000)

lc = lcm.LCM()
print "0 started"

o = utime_t()
o.utime = timestamp_now ()
lc.publish("STATE_EST_RESTART",o.encode())  
print '1 sent STATE_EST_RESTART'
time.sleep(1) # Time in seconds.

####################################################################
o = utime_t()
o.utime = timestamp_now ()
lc.publish("STATE_EST_READY",o.encode())  
print '2 sent STATE_EST_READY'
time.sleep(1) # Time in seconds.

####################################################################
m = indexed_measurement_t()
m.utime = timestamp_now ()
m.state_utime = m.utime
m.measured_dim = 4
m.z_effective = [0,0,0.85,0]
m.z_indices = [9, 10, 11, 8]
m.measured_cov_dim = 16
m.R_effective = [0.25, 0, 0, 0, 0, 0.25, 0, 0, 0, 0, 0.25, 0, 0, 0, 0, 0.75]
lc.publish("MAV_STATE_EST_VIEWER_MEASUREMENT",m.encode())  
print '3 sent MAV_STATE_EST_VIEWER_MEASUREMENT'
