#!/usr/bin/python
import os,sys
home_dir =os.getenv("DRC_BASE")
#print home_dir
sys.path.append(home_dir + "/software/build/lib/python2.7/site-packages")
sys.path.append(home_dir + "/software/build/lib/python2.7/dist-packages")


import lcm
from bot_core.pose_t import pose_t

msg = pose_t()
msg.utime = 0
msg.pos = (0, 0, 1.1)
msg.orientation = (1, 0, 0, 0)

lc = lcm.LCM()
channel = "POSE_BODY_ALT"
lc.publish(channel, msg.encode())
print "sent " + channel
print msg.pos
print msg.orientation