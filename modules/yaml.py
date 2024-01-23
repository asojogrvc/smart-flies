# Mission yaml ouput code

import numpy as np, os

from modules import uav as UAVS, coordinates as CO

def landing_Mode_to_Int(mode: str):
    match mode:
        case "None":
            return str(0)
        case "Auto":
            return str(2)

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