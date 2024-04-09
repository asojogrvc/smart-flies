import numpy as np, matplotlib.pyplot as plt

from modules import bases as BA, tasks as TS, uavs as UAVS

bases = BA.Bases()
bases.add_Base(
    "B0",
    np.array([0, 0, 0])
)
bases.add_Base(
    "B1",
    np.array([0, 3, 3])
)

towers = TS.Towers()
towers.add_Tower(
    "T1",
    np.array([1,1,1])
)
towers.add_Tower(
    "T2",
    np.array([1,2,1])
)
towers.add_Tower(
    "T3",
    np.array([2,2,1])
)

towers.add_Power_Lines([("T1", "T2"), ("T2", "T3")])

tasks = TS.Tasks()
tasks.add_Task("Task1", tower = "T1")
tasks.add_Task("Task2", pair_of_towers = ("T2", "T3"))
tasks.print()

fig = plt.figure()
axes = fig.add_subplot(111)

bases.plot(axes)
towers.plot(axes)

plt.show()