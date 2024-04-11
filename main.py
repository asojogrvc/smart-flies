import numpy as np, matplotlib.pyplot as plt, networkx as nx

from modules import bases as BA, tasks as TS, uavs as UAVS, solver as SO

# --------------------------------------------------------------------------

def create_Bases() -> BA.Bases:
    bases = BA.Bases()
    bases.add_Base(
        "B0",
        np.array([-1, -1, 0])
    )
    bases.add_Base(
        "B1",
        np.array([4, 4, 0])
    )

    return bases

def create_Towers() -> TS.Towers:
    towers = TS.Towers()
    towers.add_Tower(
        "T1",
        np.array([0,0,0])
    )
    towers.add_Tower(
        "T2",
        np.array([1,0,0])
    )
    towers.add_Tower(
        "T3",
        np.array([2,0,0])
    )
    towers.add_Tower(
        "T4",
        np.array([0,1,0])
    )
    towers.add_Tower(
        "T5",
        np.array([1,1,0])
    )
    towers.add_Tower(
        "T6",
        np.array([2,1,0])
    )
    towers.add_Tower(
        "T7",
        np.array([0,2,0])
    )
    towers.add_Tower(
        "T8",
        np.array([1,2,0])
    )
    towers.add_Tower(
        "T9",
        np.array([2,2,0])
    )

    towers.add_Power_Lines([("T1", "T2"), ("T2", "T5"), ("T4", "T5"), ("T5", "T6"), ("T5", "T8") , ("T7", "T8"), ("T8", "T9")])

    return towers

def create_UAVs() -> UAVS.UAV_Team:

    uav_team = UAVS.UAV_Team()
    uav_team.add_UAV(UAVS.UAV(
        id = 0, model = "A", base = "B0"
    ))
    uav_team.add_UAV(UAVS.UAV(
        id = 1, model = "A", base = "B0"
    ))

    return uav_team

def create_Tasks() -> TS.Tasks:
    tasks = TS.Tasks()
    tasks.add_Task("tT1", inspection_of = "T1", incompatible_IDs = [0,])
    tasks.add_Task("tT2", inspection_of = "T2", incompatible_IDs = [0,])
    tasks.add_Task("tT3", inspection_of = "T3", incompatible_IDs = [0,])
    tasks.add_Task("tT4", inspection_of = "T4", incompatible_IDs = [0,])
    tasks.add_Task("tT5", inspection_of = "T5", incompatible_IDs = [0,])
    tasks.add_Task("tS1", inspection_of = ("T5", "T8"))
    tasks.add_Task("tS2", inspection_of = ("T7", "T8"))
    tasks.add_Task("tS3", inspection_of = ("T8", "T9"))

    return tasks

# --------------------------------------------------------------------------

bases = create_Bases()
towers = create_Towers()
uav_team = create_UAVs()
tasks = create_Tasks()

problem = SO.Problem(bases, towers, tasks, uav_team)
routes = problem.solve(dynamic = False)

print(routes)


graph = problem.get_Graph()


fig = plt.figure()
axes = fig.add_subplot(131)
nx.draw_networkx(graph, ax=axes, with_labels = True)
axes1 = fig.add_subplot(132)
nx.draw_networkx(SO.get_Subgraph(graph, 0), ax=axes1, with_labels = True)
axes2 = fig.add_subplot(133)
nx.draw_networkx(SO.get_Subgraph(graph, 1), ax=axes2, with_labels = True)


fig2 = plt.figure()
axes3 = fig2.add_subplot(111)
bases.plot(axes3)
towers.plot(axes3)



plt.show()

    