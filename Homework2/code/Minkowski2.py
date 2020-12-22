import numpy as np
import math
from numpy import linalg as LA
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from matplotlib.collections import PatchCollection
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection



def getAngle(v1, v2):

	line_of_sight = v2 - v1

	# angle of vector wrt x-axis (i..e the horizontal)
	if(LA.norm(line_of_sight)== 0):
		print(v1, v2)
		print("line_of_sight", line_of_sight)
		print('divide by zero')
		return(1)
	theta = np.arccos(np.dot(line_of_sight, [1,0])/(LA.norm(line_of_sight)*LA.norm([1,0])) ) 
	theta = theta * 180/np.pi
	# print(theta) 
	return theta


def Generate_C_Space_Obs(workspace_obs, robot):

	# c-space obstacle represented by vertices
	c_space_obs = np.empty((0,2), float)

	i = 0
	j = 0
	while ( (i < len(robot)+1) & (j < len(workspace_obs)+1)):

		# add the current vertices to our c-space obstacle
		# first vertex is neg_A[0] + obs[0] so neg_A[0] is our reference point
		vertex_sum = robot[ i%len(robot) ] + workspace_obs[ j%len(workspace_obs) ]
		c_space_obs = np.append(c_space_obs, [vertex_sum], axis = 0)

		# Use % operator to ensure we wrap around if we get to the end of vertices
		# 0 % 3 = 0 and 1 % 3 = 1 
		robot_vert_angle = getAngle(robot[i%len(robot)], robot[(i+1) %len(robot)])
		obs_vert_angle = getAngle(workspace_obs[j%len(workspace_obs)], workspace_obs[(j+1)%len(workspace_obs)])


		if (robot_vert_angle < obs_vert_angle):
			# move to the next vertex of the robot
			i += 1
		elif (robot_vert_angle > obs_vert_angle):
			# move to the next vertex of the obstacle
			j += 1
		else:
			i += 1
			j += 1

	return c_space_obs

def c_space_plot_3D(c_space_obs, theta, ax):

	xvals = c_space_obs[:,0]
	yvals = c_space_obs[:,1]
	# only using one theta value for this slice
	theta_vals = [theta, theta, theta, theta]
	verts = [list(zip(xvals,yvals,theta_vals))]

	ax.add_collection3d(Poly3DCollection(verts))


def turn_robot(robot, angle):

	turned_robot = np.empty((0,2), float)
	for vert in robot:

		# move the vertex of robot by the supplied angle
		x_prime = vert[0] + 0.1 * np.cos(angle)
		y_prime = vert[1] + 0.1 * np.sin(angle) 

		new_vertex = np.array([x_prime, y_prime])

		# add it to the array of vertices
		turned_robot = np.append(turned_robot, [new_vertex], axis = 0)

	return turned_robot

if __name__ == "__main__":

	# define obstacle in terms of vertices 
	obs = np.array([[0,0], [1,2], [0,2]])

	# robot has same shape as obstacle
	neg_A = np.array([[-1,-2],[0,-2],[0,0]])

	# Angles by which to turn the robot
	theta = np.linspace(0.1,2*np.pi,100)
	

	# Create plot
	fig = plt.figure()
	ax = Axes3D(fig)
	
	# Generate a c-space obstacle for each robot rotation
	for angle in theta:
		neg_A = turn_robot(neg_A, angle)
		# print(neg_A)
		c_space_obs = Generate_C_Space_Obs(obs, neg_A)
		
		
		# add current points to th ax object
		c_space_plot_3D(c_space_obs, angle, ax)

	# Plotting properties
	ax.set_xlim3d(min(c_space_obs[:,1])*2, max(c_space_obs[:,1])*2)
	ax.set_ylim3d(min(c_space_obs[:,1])*2, max(c_space_obs[:,1])*2)
	ax.set_zlim(0,3*np.pi)

	ax.set_xlabel('X')
	ax.set_ylabel('Y')
	ax.set_zlabel('Theta (rad)')
	plt.grid(color = 'b', linestyle = '-', linewidth = 1)
	plt.show()

	
