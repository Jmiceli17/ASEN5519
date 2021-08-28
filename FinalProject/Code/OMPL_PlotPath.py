####################################################
# OMPL_PlotPath.py
# 
# Author: Joe Miceli
# 
# Description: Script for plotting an output log
# file from implementation of SST motion planning 
# algorithm in OMPL (developed for ASEN5519 final
# project)
####################################################


import sys
import numpy as np
import matplotlib.pyplot as plt

data = np.genfromtxt("SST_PATH.txt")

# Attitude quaternion portion of the state
Qx = data[:,0]
print(Qx)	#TEST
Qy = data[:,1]
Qz = data[:,2]
Qw = data[:,3]

# Angular velocity of the state
W0 = data[:,4]
W1 = data[:,5]
W2 = data[:,6]

# Controls applied at each state
U0 = data[:,7]
U1 = data[:,8]
U2 = data[:,9]

# Control duration
# This array specifies how long each control should be applied for
dt = data[:,10]
print(dt)
# total time of path
total_time = 0
step_dt = []
for i in dt:
	total_time = total_time + i
	step_dt.append(total_time)

	
print(total_time)


# The number of data points of each array (they should all be the same length)
# this is a very course approximation of how the state evolves with time, remember the path provided by OMPL
# is the state at each node (i.e. after a a control has been applied to a state for a given duration), it does
# not show what the state does during that period of control being applied
data_pts = np.linspace(1,total_time,len(Qx))


# Plot the attititude
fig = plt.figure(1)
ax = plt.gca()
plt.plot(data_pts,Qx,'r')
plt.plot(data_pts,Qy,'b')
plt.plot(data_pts,Qz,'g')
plt.plot(data_pts,Qw,'k')
ax.grid(color='y', linestyle='--')
ax.set_title('Evolution of Attitude Quaternion On SST-Generated Path', fontsize = 16)



# Plot the angular velocity
fig = plt.figure(2)
ax = plt.gca()
plt.plot(data_pts,W0,'r')
plt.plot(data_pts,W1,'b')
plt.plot(data_pts,W2,'g')
ax.grid(color='y', linestyle='--')
ax.set_title('Evolution of Angular Velocity On SST-Generated Path', fontsize = 16)



# Plot the controls
fig = plt.figure(3)
ax = plt.gca()
plt.step(step_dt,U0,'r', linewidth = 2)
plt.step(step_dt,U1,'b', linewidth = 2)
plt.step(step_dt,U2,'g', linewidth = 2)
ax.grid(color='y', linestyle='--')

ax.legend([r'$ {\tau}_{0}$', r'$ {\tau}_{1}$', r'$ {\tau}_{2}$'], fontsize = 14)
ax.set_title('SST-Generated Control Sequence', fontsize = 16)
ax.set_ylabel('Control Torque (N*m)', fontsize = 14)
ax.set_xlabel('Time (sec)', fontsize = 14)
plt.show()
