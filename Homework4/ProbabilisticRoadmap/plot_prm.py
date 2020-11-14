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

		try: 
			# If the log file has "Path Length" in it, show it on the title
			# Get metrics from the log file

			num_nodes = len(data['X'])	
			path_length = data['PATH_LENGTH'][:][-1]	# Path length is the last number in the column
			axes1.set_title('PRM GENERATED PATH\n Path Length: ' + str(path_length), fontsize=14)
			axes1.grid(color='y', linestyle='--')
			axes1.set_xlabel('X position (m)', fontsize=11)
			axes1.set_ylabel('Y position (m)', fontsize=11)

			plt.plot(data['X'], data['Y'], '-o')
		# plt.show()

		except:
			# Labels		
			axes1.set_title('GENERATED NODES', fontsize=14)
			axes1.set_xlabel('X position (m)', fontsize=11)
			axes1.set_ylabel('Y position (m)', fontsize=11)

			textstr = "Number of Nodes: " + str(num_nodes)
			axes1.text(0.9, -0.05, textstr, transform=axes1.transAxes, fontsize=11,
	        verticalalignment='top')

			axes1.grid(color='y', linestyle='--')
			plt.plot(data['X'], data['Y'], 'o')

		
		plt.show()
