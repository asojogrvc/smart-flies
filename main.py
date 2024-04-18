import numpy as np, matplotlib.pyplot as plt, networkx as nx

from modules import bases as BA, tasks as TS, uavs as UAVS, solver as SO, files as F


def plot_Routes_Abstract(routes: dict, G: nx.MultiDiGraph, axes: plt.Axes):

    colors = ["r", "g", "b"]

    edge_colors = {}

    edges = []
    for k, uav_id in enumerate(routes):
        route = routes[uav_id]

        for edge in route:

            edge_colors[edge] = colors[k]
            edges.append((edge[0], edge[1], uav_id))


    nx.draw_networkx(G, ax=axes, edgelist = edges, edge_color = edge_colors.values(), with_labels = True)

    return None

def plot_Routes(real_routes: dict, coordinates_dict: dict, axes: plt.Axes):

    colors = ["r", "g", "b"]

    j = 0
    for uav_id in real_routes:

        route = real_routes[uav_id]

        coordinates = np.zeros((len(route), 3))

        k = 0
        for point in route:
            coordinates[k,:] = coordinates_dict[point]
            k += 1

        axes.plot(coordinates[:,0], coordinates[:,1], colors[j])
        j += 1

    return None

# --------------------------------------------------------------------------

problem = F.load_Problem_from_File("./files/mission.json")

bases = problem.get_Bases()
towers = problem.get_Towers()
tasks = problem.get_Tasks()
uav_team = problem.get_UAVs()

bases.print()
tasks.print()
uav_team.print()

# --------------------------------------------------------------------------

routes = problem.solve(dynamic = False)

print("Routes", routes)

graph = problem.get_Graph()

fig2 = plt.figure()
axes3 = fig2.add_subplot(111)
bases.plot(axes3)
towers.plot(axes3)

vertices_dict = tasks.get_Task_Parsing_Dict()
for uav in uav_team:
    vertices_dict[uav.get_Base()] = [uav.get_Base()]

real_routes = SO.parse_Routes(routes , vertices_dict)
coordinates_dict = towers.get_Positions()
for name, position in bases:
    coordinates_dict[name] = position

print("Real Routes", real_routes)
plot_Routes(real_routes, coordinates_dict, axes3)

fig3 = plt.figure()
axes4 = fig3.add_subplot(111)
plot_Routes_Abstract(routes, graph, axes4)

plt.show()

    