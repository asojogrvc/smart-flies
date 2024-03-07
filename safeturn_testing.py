from modules import coordinates as CO
import numpy as np
import matplotlib.pyplot as plt

p1 = np.array([0,0])
n1 = np.array([0,1])
p2 = np.array([5, 2])
r1 = 0.5
r2 = 1

circular_orbit, clk, list_all = CO.get_Safe_Loiter_Alignment(p1, n1, p2, r1, r2)

print(clk)

fig = plt.figure()
ax = fig.add_subplot(111)
circle = plt.Circle((p2[0], p2[1]), r1, color='blue')
ax.plot(p1[0], p1[1], 'o')
for point in list_all:

    if 2 == len(point):
        plt.plot(point[0][0], point[0][1], 'o')
        plt.plot(point[1][0], point[1][1], 'o')
    else:
        plt.plot(point[0][0], point[0][1], 'o')

ax.add_patch(circle)
ax.plot(circular_orbit[:,0], circular_orbit[:,1], 'o')
#ax.plot(-5.10338822,  3.51080589, 'o')
ax.axis('equal')
plt.show()



