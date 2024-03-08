import numpy as np, matplotlib.pyplot as plt

from modules import coordinates as CO

p = np.array([5, 7, 6])
radius_vector = 5 * np.array([1, 0])
dheight = 10
clkwiseQ = True

points = CO.get_3D_Spiral_Path(p, radius_vector, clkwiseQ, 4, dheight, step_size = 1)

ax = plt.figure().add_subplot(projection='3d')
ax.plot(points[:,0], points[:,1], points[:, 2])
plt.show()