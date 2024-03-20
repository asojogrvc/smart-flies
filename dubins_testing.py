from modules import dubins as DB, coordinates as CO
import numpy as np
import matplotlib.pyplot as plt

p1 = np.array([0, 0, 5])
n1 = np.array([1, 0, 0])
p2 = np.array([-5, 5, 0])
n2 = np.array([1, 0, 0])

radius = 2
g = 20 / 100

points, _ = CO.get_Safe_Dubins_3D_Path(p1, n1, p2, n2, radius, g, step_size=0.5)

ax = plt.figure().add_subplot(projection='3d')
ax.plot(points[:,0], points[:,1], points[:,2], 'o')
ax.plot(p1[0], p1[1], p1[2], 'o')
ax.plot(p2[0], p2[1], p2[2], 'o')
ax.set_xlabel("x")
ax.set_ylabel("y")
ax.set_zlabel("z")
plt.show()
