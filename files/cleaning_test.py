#cleaning_test.py

import matplotlib.pyplot as plt

from modules import bases as BA, towers as TW, uav as UAVS, coordinates as CO

uav = UAVS.UAV()
uav.load_from_Model("dji_M210_noetic")

quit()

bases = BA.Bases()
bases.load_File("./files/bases.kml", True)

towers = TW.Towers()
towers.load_File("./files/towers.kml", True)
print(towers.get_Towers())



towers.print()

base = bases.get_Base("B1")

latlon = CO.utm2latlon(base.get_Coordinates(), base.get_UTM_Zone())
print(latlon)

bases.print()

fig = plt.figure()
ax = fig.add_subplot(111)

towers.plot(ax)
bases.plot(ax)
plt.show()