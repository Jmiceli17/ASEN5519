import numpy as np
# Convex hull of a random set of points:

# obs = np.array([[0,0], [1,2], [0,2]])

# # robot has same shape as obstacle
# neg_A = np.array([[-1,-2],[0,-2],[0,0]])

# two squares
#obs = np.array([[0,0], [0,1], [1,1], [1,0]])

# # robot has same shape as obstacle
# neg_A = np.array([[-1,-1], [-1,0], [0,0], [0,-1]])

# #Two triangles
# obs = np.array([[0,1], [0,-1], [1,0]])

# # robot has same shape as obstacle
# neg_A = np.array([[-1,-1], [-1,0], [0,0], [0,-1]])


obs = np.array([[0,0], [1,2], [0,2]])

# robot has same shape as obstacle
neg_A = np.array([[-1,-2],[0,-2],[0,0]])
# neg_A = np.array([[0,0], [1,2], [0,2]])

points = np.empty((0,2),float)
print(points)

for point in obs:
	for pt in neg_A:
		points = np.append(points,[point+pt], axis=0)

print(points)
from scipy.spatial import ConvexHull
# points = np.random.rand(30, 2)   
hull = ConvexHull(points)

# Plot it:

import matplotlib.pyplot as plt
plt.plot(points[:,0], points[:,1], 'o')
for simplex in hull.simplices:
    plt.plot(points[simplex,0], points[simplex,1], 'k-')

# We could also have directly used the vertices of the hull, which
# for 2-D are guaranteed to be in counterclockwise order:

plt.plot(points[hull.vertices,0], points[hull.vertices,1], 'r--', lw=2)
plt.plot(points[hull.vertices[0],0], points[hull.vertices[0],1], 'ro')
plt.show()