import numpy as np
import matplotlib.pyplot as plt

data = np.genfromtxt("motion_planning.csv", delimiter=",", names=["x", "y"])


plt.plot(data['x'], data['y'], color = 'b', linestyle = '-', linewidth = 2)

plt.grid(color='y', linestyle='--')


plt.show()