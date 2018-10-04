#!/usr/bin/env python
import rosbag
import numpy as np
import matplotlib.pyplot as plt
import sys
import os

args = sys.argv
print(len(args))
assert len(args)>=2, "you must specify the argument."

# get path
filename=os.path.normpath(os.path.join(os.getcwd(),args[1]))
print(filename)

# read from bag file
bag = rosbag.Bag(filename)
np_poses=None
for topic, msg, t in bag.read_messages():
    if topic=="/pose":
	np_pose=np.array([[0.0, 0.0, 0.0, 0.0, 0.0]])
        np_pose[0,0]=msg.position.x
        np_pose[0,1]=msg.position.y
        np_pose[0,2]=msg.position.z
        np_pose[0,3]=t.secs
        np_pose[0,4]=t.nsecs
        if np_poses is None:
            np_poses=np_pose
        else:
            np_poses=np.append(np_poses,np_pose,axis=0)

# reform time
start_sec=np_poses[0,3]
start_nsec=np_poses[0,4]
t=np.zeros(np_poses.shape[0],dtype='float32')
for i in range(np_poses.shape[0]):
    t[i]=(np_poses[i,3]-start_sec)+(np_poses[i,4]-start_nsec)/1000000000.0

# plot    
plt.subplot(121)
plt.title("time vs x,y")
plt.plot(t, np_poses[:,0], 'r', label="x")
plt.plot(t, np_poses[:,1], 'b', label="y")
plt.xlabel("time[s]")
plt.ylabel("vel[m/s]")
plt.legend()

plt.subplot(122)
plt.title("x vs y")
plt.plot(np_poses[:,0], np_poses[:,1], 'g')
plt.show()

bag.close()
