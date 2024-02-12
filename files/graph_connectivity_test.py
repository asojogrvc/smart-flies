import networkx as nx
import sys, os, copy, matplotlib.pyplot as plt

from modules import solver as SO, towers as TW, bases as BA, uav as UAVS, weather as WT, yaml as YAML

bases, towers, uavs, weather = YAML.load_data_from_YAML("./server/dynamic/mission_init.yaml")

problem = SO.Problem(
    "1",
    towers,
    bases,
    uavs,
    weather
)

problem.solve("abstract")

pgraph = problem.get_Graph()
tgraph = problem.get_Towers().get_Graph()

nx.draw(pgraph, with_labels = True)
plt.show()

quit()


# For DIRECTED GRAPHS. This is nice

print("Is PGraph semiconnected?: ", nx.is_semiconnected(pgraph))
print("Is PGraph weakly connected?: ", nx.is_weakly_connected(pgraph))
print("Is PGraph strongly connected?: ", nx.is_strongly_connected(pgraph))

# nx.weakly_connected_components(pgraph)
# nx.strongly_connected_components(pgraph)


# For UNDIRECTED GRAPHS. This allows for the subroute heuristic thingy
print("Is TGraph biconnected?: ", nx.is_biconnected(tgraph))
print("Is TGraph connected?", nx.is_connected(tgraph))

S = [tgraph.subgraph(c).copy() for c in nx.connected_components(tgraph)]
print(S)

fig = plt.figure()
axes = fig.add_subplot(121)
axes1 = fig.add_subplot(122)

nx.draw(S[0], ax = axes, with_labels = True, node_color = "red")
nx.draw(S[1], ax = axes1, with_labels = True, node_color = "blue")


plt.show()

