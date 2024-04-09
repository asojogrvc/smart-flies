import numpy as np, matplotlib.pyplot as plt, networkx as nx

from modules import bases as BA, tasks as TS, uavs as UAVS, solver as SO

# --------------------------------------------------------------------------

def create_Bases() -> BA.Bases:
    bases = BA.Bases()
    bases.add_Base(
        "B0",
        np.array([0, 0, 0])
    )
    bases.add_Base(
        "B1",
        np.array([0, 3, 3])
    )

    return bases

def create_Towers() -> TS.Towers:
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

    return towers

def create_UAVs() -> UAVS.UAV_Team:

    uav_team = UAVS.UAV_Team()
    uav_team.add_UAV(UAVS.UAV(
        id = 0, model = "A", base = "B0"
    ))
    uav_team.add_UAV(UAVS.UAV(
        id = 1, model = "B", base = "B1"
    ))

    return uav_team

def create_Tasks() -> TS.Tasks:
    tasks = TS.Tasks()
    tasks.add_Task("TaskT1", inspection_of = "T1", incompatible_IDs = [0,])
    tasks.add_Task("TaskS2", inspection_of = ("T2", "T3"))

    return tasks

# --------------------------------------------------------------------------

bases = create_Bases()
towers = create_Towers()
uav_team = create_UAVs()
tasks = create_Tasks()

tasks.print()

problem = SO.Problem(bases, towers, tasks, uav_team)

graph = nx.MultiDiGraph()
SO.construct_Abstract_Graph(graph, bases, towers, tasks, uav_team, "")

fig = plt.figure()
axes = fig.add_subplot(111)
nx.draw_networkx(graph, ax=axes, with_labels = True)

"""
bases.plot(axes)
towers.plot(axes)
"""

plt.show()

    