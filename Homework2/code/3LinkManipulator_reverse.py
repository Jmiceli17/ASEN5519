#---------------------------------
# ASEN 5519 Homework 2
#
# Reverse kinematic model for a 
# 3-link planar manipulator
# 
# Author:
# 	Joe Miceli
# 	Oct2020
# 
# Input:
#	Desired x,y coordinates
# 	3 link lengths (m)
#
# Output:
#	Visualization of valid configurations
#	and positions of end points of 
# 	last links
#---------------------------------

import numpy as np
import matplotlib.pyplot as plt
import ReverseKine as rk

# Get endpoint of 1st link
def get_first_endpt(theta1,l1):
	c1 = np.cos(theta1)
	s1 = np.sin(theta1)
	# Transformation matrix from global frame to frame of 1st link
	T1 = [[c1, -s1, 0],
	      [s1, c1,  0],
	      [0,  0,   1]]

	# translation of end point in frame of 1st link
	trans_of_a = [[1, 0, l1], [0, 1, 0], [0, 0, 1]]

	empty_vec = [0,0,1]

	prod = np.matmul(np.matmul(T1, trans_of_a), empty_vec)

	x1 = prod[0]
	y1 = prod[1]
	return [x1, y1], T1


# Get endpoint of 2nd link
def get_second_endpt(theta2, l2, T1):
	c2 = np.cos(theta2)
	s2 = np.sin(theta2)
	# Transformation matrix from frame of 1st link to frame of 2nd link
	T2 = [[c2, -s2, l2],
	      [s2, c2,  0],
	      [0,  0,   1]]

	# translation of end point in frame of 2nd link
	trans_of_a = [[1, 0, l2], [0, 1, 0], [0, 0, 1]]

	# needed to make result a 3x1
	empty_vec = [0,0,1]


	prod = np.matmul(np.matmul(np.matmul(T1, T2), trans_of_a), empty_vec)

	x2 = prod[0]
	y2 = prod[1]
	return [x2, y2], T2



# Get endpoint of 3rd link
def get_third_endpt(theta3, l3, T1, T2):
	c3 = np.cos(theta3)
	s3 = np.sin(theta3)
	# Transformation matrix from frame of 1st link to frame of 2nd link
	T3 = [[c3, -s3, l3],
	      [s3, c3,  0],
	      [0,  0,   1]]

  	# translation of end point in frame of 3rd link
	trans_of_a = [[1, 0, l3], [0, 1, 0], [0, 0, 1]]

	# needed to make result a 3x1
	empty_vec = [0,0,1]

	prod = np.matmul(np.matmul(np.matmul(np.matmul(T1, T2), T3), trans_of_a), empty_vec)

	x3 = prod[0]
	y3 = prod[1]
	return [x3, y3], T3



def plot_3link_planar(pt1, pt2, pt3, solution_number):
	plt.figure()
	pts = np.array([[0,0], pt1, pt2, pt3])
	x, y = pts.T
	plt.plot(x,y, linestyle='-', marker = 'o', color = 'k', linewidth = 3)
	plt.xlabel('X position (m)')
	plt.ylabel('Y positon (m)')

	axes = plt.gca()
	title = 'SOLUTION NUMBER: ' + str(solution_number)
	axes.set_title(title)
	axes.set_xlim([1.5*min(x),1.5*max(x)])
	axes.set_ylim([1.5*min(y),1.5*max(y)])
	# Annotate the end point of the robot
	axes.annotate( str([x[-1], y[-1]]) , xy = ([x[-1], y[-1]]) ) 
	plt.grid(color = 'b', linestyle = '-', linewidth = 1)
	plt.show()

if __name__ == "__main__":
	
	# theta1 = input("ENTER THETA 1 (angle of first link wrt. global frame in deg): ")
	# theta2 = input("ENTER THETA 2 (angle of first link wrt. link 1 in deg): ")
	# theta3 = input("ENTER THETA 3 (angle of first link wrt. link 2 in deg): ")



	l1 = input("ENTER L_1 (the length of the first link in m): ")
	l2 = input("ENTER L_2 (the length of the second link in m): ")
	l3 = input("ENTER L_3 (the length of the third link in m): ")

	X_desired = input("ENTER DESIRED X (the x component of the desired position in m): ")
	Y_desired = input("ENTER ENTER DESIRED Y (the y component of the desired position in m): ")


	# Check inputs
	# check_thetas = [theta1, theta2, theta3]
	# for t in check_thetas:
	# 	if (isinstance(t, float) or isinstance(t, int)):
	# 		continue
	# 	else:
	# 		print('ERROR: ", t, "is not a valid scalar. Please try again.')
	# 		exit(1)

	check_des = [X_desired,Y_desired]
	for des in check_des:
		if (isinstance(des, float) or isinstance(des, int)):
			continue
		else:
			print('ERROR: ', des, 'is not a valid scalar. Please try again.')
			exit(1)

	check_links = [l1, l2, l3]
	for L in check_links:
		if (isinstance(L, float) or isinstance(L, int)):
			continue
		elif (L <= 0):
			print('ERROR: Link length must be a positive value.')
			print('Please try again.')
			exit(1)

		else:
			print('ERROR: ', L, 'is not a valid scalar. Please try again.')
			exit(1)


	
	# Find valid thetas that achieve the desired position
	valid_sol = rk.ReverseKine(X_desired, Y_desired, l1, l2, l3)
	i = 0
	for theta1, theta2, theta3 in valid_sol:

		# Get the solution number (used for figure title)
		solution_number = i

		# Convert to radians
		theta1 = theta1*np.pi/180
		theta2 = theta2*np.pi/180
		theta3 = theta3*np.pi/180

		# Form kinematic chain to get position of endpt of link 3 in global frame
		# [x,y,1] = T1*T2*T3*T4*[0,0,1]'
		first_pt, T1 = get_first_endpt(theta1,l1)
		secnd_pt, T2 = get_second_endpt(theta2, l2, T1)
		third_pt, T3 = get_third_endpt(theta3, l3, T1, T2)

		plot_3link_planar(first_pt, secnd_pt, third_pt, solution_number)
		#print('POSITION OF END POINT OF 1st LINK (m): ', first_pt)
		#print('POSITION OF END POINT OF 2nd LINK (m): ', secnd_pt)
		print('POSITION OF END POINT OF 3rd LINK (m): ', third_pt)

		i = i+1