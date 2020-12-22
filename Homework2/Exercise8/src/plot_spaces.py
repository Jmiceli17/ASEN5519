import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import ConvexHull



# Get data from log files
c_space_data = np.genfromtxt("c_space.csv", delimiter=",", names=True)
w_space_data = np.genfromtxt("workspace.csv", delimiter=",", names=True)


# FIGURE FOR CONFIGURATION SPACE
# here we're plotting t1 and t2 coordinates of obstacles
fig1 = plt.figure(1)
axes1 = plt.gca()
# plot points as blue circles
plt.plot(c_space_data['Theta1'], c_space_data['Theta2'], 'bo', linewidth = 1)

# Labels
axes1.set_title('Discretized Configuration Space\n for link_1 = 1(m) and link_2 = 1(m)')
axes1.set_xlabel('Theta1 (rad)')
axes1.set_ylabel('Theta1 (rad)')

# Range for c-space
axes1.set_xlim([-np.pi,np.pi])
axes1.set_ylim([-np.pi,np.pi])

axes1.grid(color='y', linestyle='--')
#fig1.show()



# FIGURE FOR WORKSPACE
# here we're plotting x and y coordinates of obstacles
fig2 = plt.figure(2)
axes2 = plt.gca()
plt.plot(w_space_data['X'], w_space_data['Y'], 'go', linewidth = 1)


# Labels
axes2.set_title('Discretized Workspace\n for link_1 = 1(m) and link_2 = 1(m)')
axes2.set_xlabel('X (m)')
axes2.set_ylabel('Y (m)')

# Range for c-space
axes2.set_xlim([-5,5])
axes2.set_ylim([-5,5])

axes2.grid(color='y', linestyle='--')
plt.show()