import numpy as np, json

from modules import bases as BA, tasks as TS, uavs as UAVS, solver as SO

def load_Problem_from_File(file_path: str) -> SO.Problem:

    f = open(file_path)
    mission = json.load(f)
    f.close() 

    bases = BA.Bases()
    for name in mission["Bases"]:
        bases.add_Base(
            name,
            np.array(mission["Bases"][name])
        )

    towers = TS.Towers()
    for name in mission["Towers"]["List"]:
        towers.add_Tower(
            name,
            np.array(mission["Towers"]["List"][name])
        )
    for power_line in mission["Towers"]["Lines"]:
        towers.add_Power_Lines((power_line[0], power_line[1]))

    uavs = UAVS.UAV_Team()
    for id_ in mission["UAVs"]:
        uavs.add_UAV(UAVS.UAV(
            id = id_, model = mission["UAVs"][id_]["Model"], base = mission["UAVs"][id_]["Base"]
        ))

    tasks = TS.Tasks()
    for name in mission["Tasks"]:

        try:
            tasks.add_Task(
                name,
                inspection_of = mission["Tasks"][name]["inspection_of"],
                incompatible_IDs = mission["Tasks"][name]["incompatible_IDs"]
            )
        except:
            tasks.add_Task(
                name,
                inspection_of = mission["Tasks"][name]["inspection_of"]
            )


    return SO.Problem(bases, towers, tasks, uavs)