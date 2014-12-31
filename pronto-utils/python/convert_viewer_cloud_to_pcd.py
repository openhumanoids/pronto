import csv
import sys
import numpy as np

fname = sys.argv[1]
ins = open(fname , "r" )
array = []
count=0
for line in ins:
    if (line[0:2] == "%%"):
      continue
    y = line.split(" ")
    q = []
    for d in y:
      q.append(float(d))

    dist_range = np.sqrt( pow( q[0]-q[3],2) + pow(q[1]-q[4],2) + pow(q[2]-q[5],2) )

    if ((dist_range < 27) and (dist_range > 3)): 
      array.append( q )
    count=count+1
    if (count%100000 ==0):
      print str(count) + " points read"
ins.close()

print str(len(array)) + " points written"

target = open( fname + ".pcd", 'w')
target.write("# .PCD v.7 - Point Cloud Data file format\n")
target.write("VERSION .7\n")
target.write("FIELDS x y z\n")
target.write("SIZE 4 4 4\n")
target.write("TYPE F F F\n")
target.write("COUNT 1 1 1\n")
target.write("WIDTH "+str(len(array))+"\n")
target.write("HEIGHT 1\n")
target.write("VIEWPOINT 0 0 0 1 0 0 0\n")
target.write("POINTS "+str(len(array))+"\n")
target.write("DATA ascii\n")
for line in array:
  target.write(str(line[0]) + " " + str(line[1]) + " " + str(line[2]) + "\n")
target.close()

