import numpy as np

# X component of kinematic equation
def x_reverseKine(t1, t2, t3, l1, l2, l3):
	x = (l1*np.cos(t1))+(l2*np.cos(t1+t2))+(l3*np.cos(t1+t2+t3))
	return x

# Y component of kinematic equation
def y_reverseKine(t1,t2,t3, l1, l2, l3):
	y = (l1*np.sin(t1))+(l2*np.sin(t1+t2))+(l3*np.sin(t1+t2+t3))
	return y

# ----------------------------------------
# X_des - x coordinate of desired position
# Y_des - y coordinate of desired position
# l1 - length of arm 1
# l2 - length of arm 2
# l3 - length of arm 3
# ----------------------------------------

def ReverseKine(X_des, Y_des, l1, l2, l3):
	# Values from Homework2 Problem 3
	#X_des = 0
	#Y_des = 4
	valid_sol = []

	# Loop over all possible values of theta (incrementing by 1)
	for theta1 in range(0,360):
		print("searching for solutions...")
		for theta2 in range(0,360):
			for theta3 in range(0,360):
				t1 = theta1*np.pi/180
				t2 = theta2*np.pi/180
				t3 = theta3*np.pi/180

				# Check if it provides a solution close to what we're looking for
				if ((abs(X_des-x_reverseKine(t1,t2,t3, l1, l2, l3)) <= 1e-2) & (abs(Y_des-y_reverseKine(t1,t2,t3, l1, l2, l3)) <= 1e-2)):
						valid_sol.append([theta1, theta2, theta3])
						print("Valid Solution Found!")
						print("theta1: ", theta1)
						print("theta2: ", theta2)
						print("theta3: ", theta3)
						print("Continuing the search...")

	if (not valid_sol):
		print("--No solutions found for the desired position and selected robot configuration.--")
		print("--Please try again--")
		exit()



	else:
		print(len(valid_sol), " solutions found:")
		print(valid_sol)

	return valid_sol

# Valid_sol = [[12, 219, 237], [66, 246, 211], [76, 256, 208], [99, 279, 207], [111, 286, 210], [114, 287, 211], [139, 287, 222], [143, 286, 224], [162, 279, 234]]
# for t1 : [0,180]

# Valid_sol = [[12, 219, 237], [18, 81, 126], [37, 74, 136], [41, 73, 138], [66, 73, 149], [66, 246, 211], [69, 74, 150], [76, 256, 208], [81, 81, 153], [99, 279, 207], [104, 104, 152], [111, 286, 210], [114, 114, 149], [114, 287, 211], [139, 287, 222], [143, 286, 224], [162, 279, 234], [168, 141, 123], [208, 256, 256], [228, 246, 263], [231, 141, 96], [309, 219, 264], [312, 114, 97], [332, 104, 104]]