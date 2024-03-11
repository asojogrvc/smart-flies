import numpy as np, matplotlib.pyplot as plt

from modules import coordinates as CO

p = np.array([0, 0, 0])
n = np.array([1, 1])
radius = 2
dheight = 2
clkwiseQ = True

points = CO.get_3D_Spiral_Path(p, n, radius, clkwiseQ, 4, dheight, step_size = 0.1)

print(points)

ax = plt.figure().add_subplot(projection='3d')
ax.plot(points[:,0], points[:,1], points[:, 2])
ax.set_xlabel("x")
ax.set_ylabel("y")
ax.set_zlabel("z")
plt.show()