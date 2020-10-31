import numpy as np
from numpy import linalg as LA
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from matplotlib.collections import PatchCollection


def getAngle(v1, v2):

	# v2 - v1
	#line_of_sight = substractVertices(v2,v1)

	line_of_sight = v2 - v1

	# angle of vector wrt x-axis (i..e the horizontal)
	if(LA.norm(line_of_sight)== 0):
		print(v1, v2)
		print("line_of_sight", line_of_sight)
		print('divide by zero')
		return(1)
	theta = np.arccos(np.dot(line_of_sight, [1,0])/(LA.norm(line_of_sight)*LA.norm([1,0])) ) 
	#theta = theta * 180/np.pi
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
		print("robot: ", i, "+ workspace_obs: ", j)
		c_space_obs = np.append(c_space_obs, [vertex_sum], axis = 0)

		# Use % operator to ensure we wrap around if we get to the end of vertices
		# 0 % 3 = 0 and 1 % 3 = 1 
		robot_vert_angle = getAngle(robot[i%len(robot)], robot[(i+1) %len(robot)])
		obs_vert_angle = getAngle(workspace_obs[j%len(workspace_obs)], workspace_obs[(j+1)%len(workspace_obs)])
		print(robot_vert_angle)
		print(obs_vert_angle)

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

def c_space_plot(c_space_obs):
	fig, ax = plt.subplots()
	poly = Polygon(c_space_obs)
	patches = []
	patches.append(poly)

	p = PatchCollection(patches, alpha=0.4)

	ax.add_collection(p)
	plt.ylim(min(c_space_obs[:,1])*2, max(c_space_obs[:,1])*2)
	plt.xlim(min(c_space_obs[:,1])*2, max(c_space_obs[:,1])*2)

	plt.grid(color = 'b', linestyle = '-', linewidth = 1)
	plt.show()


# def test_sum(A, B):
# 	for 


if __name__ == "__main__":

	# define obstacle in terms of vertices 
	#obs = {[0,0];[0,2];[1,2]}
	obs = np.array([[0,0], [1,2], [0,2]])

	# robot has same shape as obstacle
	neg_A = np.array([[-1,-2],[0,-2],[0,0]])

	c_space_obs = Generate_C_Space_Obs(obs, neg_A)
	print(c_space_obs)

	c_space_plot(c_space_obs)

