import matplotlib.pyplot as plt

from modules import bases as BA, tasks as TS, uavs as UAVS, solver as SO, files as F

problem = F.load_Problem_from_File("./files/ATLAS_U3.json")

bases = problem.get_Bases()
towers = problem.get_Towers()

fig = plt.figure()
axes = fig.add_subplot(111)
towers.plot(axes)
bases.plot(axes)
plt.show()