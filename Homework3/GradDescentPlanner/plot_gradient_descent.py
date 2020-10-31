import sys
import numpy as np
import matplotlib.pyplot as plt



if __name__ == "__main__":

	if (len(sys.argv) < 3):
		print("ERROR")
		print("USAGE: python plot_gradient_descent.py <Path To GradientDescent.csv> <Path to VectorField.csv>")
	
	else:
		grad_descent_file = sys.argv[1]
		vec_field_file = sys.argv[2]

		# ==== GET DATA FROM LOG FILES ====
		# data = np.genfromtxt("GradientDescent.csv", delimiter=",", names=True)
		# vec_field_data = np.genfromtxt("VectorField.csv", delimiter=",", names=True)

		data = np.genfromtxt(grad_descent_file, delimiter=",", names=True)
		vec_field_data = np.genfromtxt(vec_field_file, delimiter=",", names=True)

		# ==== PLOT GRADIENT AND PATH ==== 
		fig1 = plt.figure(1)
		axes1 = plt.gca()

		plt.quiver(data['X'], data['Y'], data['U'], data['V'], linewidth = 0.5)

		# Labels
		axes1.set_title('Gradient and Path Taken by Robot in 2 Dimensional Space')
		axes1.set_xlabel('X position (m)')
		axes1.set_ylabel('Y position (m)')

		# Range for w-space
		#axes1.set_xlim([0,11])
		#axes1.set_ylim([-2,2])

		axes1.grid(color='y', linestyle='--')


		# ==== PLOT THE PATH ==== 
		fig2 = plt.figure(2)
		axes2 = plt.gca()

		plt.plot(data['X'], data['Y'], linewidth = 2)

		# Labels
		axes2.set_title('Path Taken by Robot')
		axes2.set_xlabel('X position (m)')
		axes2.set_ylabel('Y position (m)')

		# Range for w-space
		#axes2.set_xlim([0,11])
		#axes2.set_ylim([-2,2])

		axes2.grid(color='y', linestyle='--')


		# ==== PLOT THE VECTOR FIELD FROM THE GRADIENT OF THE POTENTIAL FUNCTION ==== 
		fig3 = plt.figure(3)
		axes3 = plt.gca()

		plt.quiver(vec_field_data['X'], vec_field_data['Y'], vec_field_data['U'], vec_field_data['V'], linewidth = 0.5)
		plt.plot(data['X'], data['Y'], linewidth = 2)

		# Labels
		axes3.set_title('Gradient of Potential Function')
		axes3.set_xlabel('X position (m)')
		axes3.set_ylabel('Y position (m)')

		# Range for w-space
		#axes1.set_xlim([0,11])
		#axes1.set_ylim([-2,2])

		axes3.grid(color='y', linestyle='--')



		plt.show()