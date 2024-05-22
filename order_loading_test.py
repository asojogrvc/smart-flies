import networkx as nx, matplotlib.pyplot as plt

from modules import bases as BA, tasks as TS, uavs as UAVS, solver as SO, files as F

problem = F.load_Problem_from_File("./files/ATLAS_EASY.json")

bases = problem.get_Bases()
towers = problem.get_Towers()
tasks = problem.get_Tasks()
tasks.print()
uav_team = problem.get_UAVs()

abstract_G = nx.MultiDiGraph()
abstract_G = SO.construct_Abstract_Graph(abstract_G, bases, towers, tasks, uav_team)

G1 = SO.get_Subgraph(abstract_G, uav_team.get_List()["0"])
G1.remove_node("B0")

# Gotta delete those that contain bases and 
paths = list(nx.all_simple_edge_paths(G1, "tT1", "tT2")) # Up to a certain length, there sould be no need to delete those paths as they to long for optimality

print(paths, len(paths))

nx.draw_networkx(G1, with_labels = True)
plt.show()
