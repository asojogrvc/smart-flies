import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

from modules import bases as BA, tasks as TS, uavs as UAVS, solver as SO, files as F

problem = F.load_Problem_from_File("./files/eil51.json")

bases = problem.get_Bases()
towers = problem.get_Towers()

fig = plt.figure()
axes = fig.add_subplot(111)
towers.plot(axes)
bases.plot(axes)

axes.tick_params(left=True, bottom=True, labelleft=True, labelbottom=True, labelsize=15)
axes.grid()
#axes.set_xlim(0, 650)
#axes.set_ylim(0, 550)
axes.set_xlabel("x [m]", fontsize=16)
axes.set_ylabel("y [m]", fontsize=16)
fig.tight_layout()

legend_handles = [Line2D([0], [0], marker='o', color='w', label='Bases',
                          markerfacecolor='r', markersize=9),
                  Line2D([0], [0], marker='o', color='k', label='PLN',
                          markerfacecolor='C0', markersize=9)]

axes.legend(handles = legend_handles, fontsize = 16)

plt.show()