import networkx as nx, matplotlib.pyplot as plt

from modules import bases as BA, tasks as TS, uavs as UAVS, solver as SO, files as F

problem = F.load_Problem_from_File("./files/ATLAS_EASY.json")

bases = problem.get_Bases()
towers = problem.get_Towers()
tasks = problem.get_Tasks()
tasks.print()
print(tasks.get_Complex_Tasks())
uav_team = problem.get_UAVs()

routes = problem.solve(dynamic = False, auto_uav_disabling = False, cost_function = "mtm")

sGs = problem.get_Subgraphs()
ssGs = problem.get_Simplified_Subgraphs()

# Gotta delete those that contain bases and 
paths = list(nx.all_simple_edge_paths(sGs["0"], "tT1", "tT2")) # Up to a certain length, there sould be no need to delete those paths as they to long for optimality

print(paths, len(paths))

nx.draw_networkx(ssGs["0"], with_labels = True)
plt.show()
