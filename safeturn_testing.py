from modules import coordinates as CO
import numpy as np
import matplotlib.pyplot as plt

p1 = np.array([0,0])
n1 = np.array([0,1])
p2 = np.array([5,0])
r1 = 0.5
r2 = 0.6

circular_orbit, _ = CO.get_Safe_Loiter_Alignment(p1, n1, p2, r1, r2)


circle1 = plt.Circle((p1[0], p1[1]), r1, color='r')
circle2 = plt.Circle((p2[0], p2[1]), r2, color='blue')
plt.plot(circular_orbit[:,0], circular_orbit[:,1], 'o')
plt.plot(p1[0], p1[1], 'o')
plt.plot(p2[0], p2[1], 'o')
plt.axis('equal')
plt.show()



