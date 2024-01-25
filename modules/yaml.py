# Mission yaml ouput code

import numpy as np, os, yaml, copy 

from modules import bases as BA, towers as TW, uav as UAVS, weather as WT, coordinates as CO, solver as SO

def landing_Mode_to_Int(mode: str):
    match mode:
        case "None":
            return str(0)
        case "Auto":
            return str(2)
        
def int_to_Landing_Mode(mode_int: str):
    match mode_int:
        case 0:
            return "None"
        case 2:
            return "Auto"


yaml_version = 3
f_id = "/gps"
mode_yaw = 2
mode_gimbal = 0
idle_vel = 5.0

def print_Header(file):
    file.write("version: "+str(yaml_version)+"\n")
    file.write("frame_id: "+f_id+"\n\n\n")

def print_Description(file, text: str):
    file.write("description: \""+text+"\"\n")

def print_Route(file, uav: UAVS.UAV, utmZone: tuple):
    
    file.write("  - name: \"Inspection_"+uav.get_ID()+"_"+uav.missionSettings["Base"]+"\"\n")
    file.write("    uav: \""+uav.get_ID()+"\"\n")
    file.write("    wp:\n")

    for wp in uav.waypoints:

        latlon = CO.utm2latlon(wp[0], utmZone)
        #latlon[0], latlon[1] = latlon[1], latlon[0] 

        point_str = np.array2string(latlon, separator=", ")
        # This should flip to the correct latlon.
        file.write("      - {pos: "+point_str+", action: "+str(wp[1])+"}\n")

    file.write("    attributes:\n\n")
    file.write("      mode_landing: " + landing_Mode_to_Int(uav.missionSettings["Landing Mode"])+ "\n\n")
    file.write("      mode_yaw: "+str(mode_yaw)+ "\n\n")
    file.write("      mode_gimbal: "+str(mode_gimbal)+ "\n\n")
    file.write("      idle_vel: "+str(idle_vel)+ "\n\n")

def print_Routes(file, uavs: UAVS.UAV_Team, utmZone: tuple):

    file.write("route:\n")

    for uav in uavs:
        print_Route(file, uav, utmZone)
        file.write("\n\n")

def save_Mission(file_path, uavs: UAVS.UAV_Team, utmZone: tuple):

    if os.path.isfile(file_path):
        print("File already exists. Overwritting")

    with open(file_path, 'w') as f:

        print_Header(f)
        print_Description(f, "")
        print_Routes(f, uavs, utmZone)

    f.close()

    print("File Saved!")

    return None

def waypoint_to_YAML(uavs: UAVS.UAV_Team) -> dict:

        wps = {}

        wps["version"] = 3
        wps["frame_id"] = "/gps"

        wps["description"] = ""

        route_list = []

        for uav in uavs:

            route_dict = {}

            route_dict["name"] = "Inspection_"+uav.get_ID()+"_"+uav.missionSettings["Base"]
            route_dict["uav"] = uav.get_ID()

            wp_list = []

            for wp in uav.waypoints:
                wp_list.append({"pos": wp[0], "action": wp[1]})

            route_dict["wp"] = wp_list

            route_dict["attributes"] = {
                "mode_landing": 2,
                "mode_yaw":2,
                "mode_gimbal:":0,
                "idle_vel": 5.0}
            
            route_list.append(route_dict)

        wps["route"] = route_list

        return wps

def load_data_from_YAML(file_path: str) -> tuple[BA.Bases, TW.Towers, UAVS.UAV_Team, WT.Weather]:

    # Initialize data
    bases = BA.Bases()
    towers = TW.Towers()
    uavs = UAVS.UAV_Team()
    weather = WT.Weather()

    # Open YAML stream from the file path and load it as a Python object
    f = open(file_path, "r")
    mission_init = yaml.load(f, Loader = yaml.Loader)
    f.close()

    print(mission_init)

    tower_height = mission_init["settings"]["mission"][0]["tower"]

    # Load the UAVs
    k = 0
    for uav_dict in mission_init["settings"]["devices"]:

        uav = UAVS.UAV()
        uav.load_from_Model(uav_dict["device_category"], str(uav_dict["device_id"]))
        uav.missionSettings["Base"] = "B"+str(k)
        uav.missionSettings["Nav. speed"] = uav_dict["speed_navegation"]
        uav.missionSettings["Insp. speed"] = uav_dict["speed_mission"]
        uav.missionSettings["Landing Mode"] = int_to_Landing_Mode(uav_dict["landing_mode"])
        geom = mission_init["settings"]["mission"][k]
        uav.missionSettings["Insp. height"] = geom["height"]
        uav.missionSettings["Insp. horizontal offset"] = geom["offset"]

        # Update from the other pair
        uav.missionSettings['Tower distance'] = float(np.sqrt(
            uav.missionSettings['Insp. height']**2 + uav.missionSettings['Insp. horizontal offset']**2))
        
        temp = uav.missionSettings['Insp. horizontal offset']
        if temp == 0:
            uav.missionSettings['Cam. angle'] = 90
        else:
            uav.missionSettings['Cam. angle'] = float(np.rad2deg(np.arctan(
                uav.missionSettings['Insp. height'] / temp)))

        uavs.add_UAV(copy.deepcopy(uav))

        coords = np.array([mission_init["bases"][k]["latitude"], mission_init["bases"][k]["longitude"], 0])
        CO.update_Height_Online(coords)
        coords = CO.latlon2utm(coords)

        bases.add_Base(BA.Base(
            "B"+str(k),
            coords[0],
            coords[1]
        ))

        k += 1
    
    base0 = bases.get_Base("B0")
    weather.update_Online(CO.utm2latlon(base0.get_Coordinates(), base0.get_UTM_Zone()))


    paths = []

    for loc in mission_init["settings"]["loc"]:
        firstQ = True
        for point in loc["items"]:

            if True == firstQ:
                path = np.array([[point["latitude"], point["longitude"], 0.0]])
                firstQ = False
            else: path = np.concatenate((path, np.array([[point["latitude"], point["longitude"], 0.0]])))

        paths.append(path)

    print(paths)

    towers.load_from_Arrays(paths, True)

    return bases, towers, uavs, weather