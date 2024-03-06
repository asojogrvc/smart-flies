from modules import coordinates as CO
import numpy as np, matplotlib.pyplot as plt

p1 = np.array([0,0])
r1 = 0.25
p2 = np.array([2,0])
r2 = 0.75

points = CO.find_Tangents_to_2Circles(p1, r1, p2, r2)

circle1 = plt.Circle((p1[0], p1[1]), r1, color='r')
circle2 = plt.Circle((p2[0], p2[1]), r2, color='blue')

fig, ax = plt.subplots() # note we must use plt.subplots, not plt.subplot
# (or if you have an existing figure)
# fig = plt.gcf()
# ax = fig.gca()

ax.add_patch(circle1)
ax.add_patch(circle2)

for point in points:

    if 2 == len(point):
        plt.plot(point[0][0], point[0][1], 'o')
        plt.plot(point[1][0], point[1][1], 'o')
    else:
        plt.plot(point[0][0], point[0][1], 'o')

plt.show()