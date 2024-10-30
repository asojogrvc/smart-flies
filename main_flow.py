from modules import bases as BA, towers as TW, uav as UAVS, solver as SO, yaml as YAML, weather as WT
import json, matplotlib.pyplot as plt

jsonQ = True

if jsonQ:
    f = open("./files/mission.json")
    mission = json.load(f)
    f.close()
    bases, towers, uavs, weather, mode, id, parameters =  YAML.load_data_from_JSON(mission)

else:
    bases = BA.Bases()
    bases.load_File("./files/bases.kml", True)
    towers = TW.Towers()
    towers.load_File("./files/.towers.kml", True)
    uavs = UAVS.UAV_Team()
    uavs.load_File("./files/UAV_Team.xml")

    weather = WT.Weather()

    id = "0"
    mode = 0

    parameters = {}

problem = SO.Problem(id, towers, bases, uavs, weather, mode, Parameters = parameters)

fig = plt.figure()
ax = fig.add_subplot(111)

bases.plot(ax)
towers.plot(ax)

#plt.show()

fastQ = True
solvedQ = problem.solve("", fastQ)

if False == solvedQ:
    print("Problem is infeasible")
else:
    fig = plt.figure()
    axes = fig.add_subplot(111)

    bases.plot(axes)
    towers.plot(axes)
    uavs.plot_Routes(axes)

    uavs.compute_Team_Waypoints(mode, towers, bases, weather.get_Wind_Direction())
    
    for uav in uavs:

        print(uav.get_ID()+":\n")
        uav.waypoints.print()
        print("---- \n")
    
    YAML.save_Mission("./mission.yaml", id, uavs)

    plt.show()

