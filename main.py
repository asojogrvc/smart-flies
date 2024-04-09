import numpy as np, matplotlib.pyplot as plt

from modules import bases as BA, uavs as UAVS

bases = BA.Bases()
bases.add_Base(
    "B0",
    np.array([0,0,0])
)
bases.add_Base(
    "B1",
    np.array([0,3,3])
)

bases.print()

fig = plt.figure()
axes = fig.add_subplot(111)

bases.plot(axes)
plt.show()