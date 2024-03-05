from modules import dubins as DB
import numpy as np
import matplotlib.pyplot as plt

p1 = np.array([1,0])
n1 = np.array([1,1]) / np.sqrt(2)
p2 = np.array([-1,0])
n2 = np.array([0,-1])


points, _, _, _ = DB.plan_dubins_path(p1, n1, p1, n2, 1)

plt.axline((p1[0], p1[1]), slope= n1[1] / n1[0])
plt.axline((p2[0], p2[1]), slope= n2[1] / n2[0])
plt.plot(points[:,0], points[:,1], 'o')
plt.plot(p1[0], p1[1], 'o')
plt.plot(p2[0], p2[1], 'o')
plt.axis('equal')
plt.show()
