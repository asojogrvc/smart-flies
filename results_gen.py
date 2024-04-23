import networkx as nx, matplotlib.pyplot as plt
from modules import files as F


n_towers = 10
p_connections = 0.001

G = F.generate_random_PLN(n_towers, p_connections)
pos = nx.planar_layout(G, 200, [0, 0])
print(pos)
nx.draw_networkx(G, pos = pos)
plt.show()