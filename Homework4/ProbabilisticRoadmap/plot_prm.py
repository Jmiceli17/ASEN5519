import sys
import numpy as np
import matplotlib.pyplot as plt



if __name__ == "__main__":

	if (len(sys.argv) < 2):
		print("ERROR")
		print("USAGE: python plot_prm.py <Path To PRM.csv>")
	
	else:
		prm_path = sys.argv[1]

		# ==== GET DATA FROM LOG FILES ====

		data = np.genfromtxt(prm_path, delimiter=",", names=True)

		#data = np.genfromtxt("PRM.csv", delimiter=",", names=True)


		# ==== PLOT PATH ==== 
		fig1 = plt.figure(1)
		axes1 = plt.gca()

		plt.plot(data['X'], data['Y'], 'o')

		# Get the number of nodes in the log file
		num_nodes = len(data['X'])	

		# Labels
		#axes1.set_title('Path Taken by Robot in 2 Dimensional Space \n(Workspace 1 from Exercise 7 of Homework 1)')
		axes1.set_xlabel('X position (m)')
		axes1.set_ylabel('Y position (m)')

		textstr = "Number of Nodes: " + str(num_nodes)
		axes1.text(0.9, -0.05, textstr, transform=axes1.transAxes, fontsize=11,
        verticalalignment='top')

		# Range for w-space
		#axes1.set_xlim([0,11])
		#axes1.set_ylim([-2,2])

		axes1.grid(color='y', linestyle='--')

		plt.show()
