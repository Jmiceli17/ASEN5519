import sys
import numpy as np
import matplotlib.pyplot as plt



if __name__ == "__main__":

	if (len(sys.argv) < 2):
		print("ERROR")
		print("USAGE: python plot_wavefront_path.py <Path To WaveFrontPlanner.csv>")
	
	else:
		wavefront_path_file = sys.argv[1]

		# ==== GET DATA FROM LOG FILES ====

		data = np.genfromtxt(wavefront_path_file, delimiter=",", names=True)

		# ==== PLOT PATH ==== 
		fig1 = plt.figure(1)
		axes1 = plt.gca()

		plt.plot(data['X'], data['Y'], linewidth = 1.5)

		# Labels
		axes1.set_title('Path Taken by Robot in 2 Dimensional Space \n(Workspace 1 from Exercise 7 of Homework 1)')
		axes1.set_xlabel('X position (m)')
		axes1.set_ylabel('Y position (m)')

		# Range for w-space
		#axes1.set_xlim([0,11])
		#axes1.set_ylim([-2,2])

		axes1.grid(color='y', linestyle='--')

		plt.show()
