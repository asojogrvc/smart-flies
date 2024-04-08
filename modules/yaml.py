# Mission yaml ouput code

import numpy as np, os, yaml, copy, json

from modules import bases as BA, tasks as TW, weather as WT, coordinates as CO, workers as UAVS

class NumpyArrayEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        return json.JSONEncoder.default(self, obj)

def landing_Mode_to_Int(mode: str) -> int:
    """
    Takes a landing mode as a str and converts it to its int equivalent
    """
    match mode:
        case "None":
            return str(0)
        case "Auto":
            return str(2)
        case _: 
            return str(0)
        
def int_to_Landing_Mode(mode_int: int) -> str:
    """
    Takes a landing mode as a int and converts it to its str equivalent
    """
    match mode_int:
        case 0:
            return "None"
        case 2:
            return "Auto"
        case _:
            return "None"

yaml_version = 3
f_id = "/gps"
mode_yaw = 2
mode_gimbal = 0
idle_vel = 5.0

def print_Header(file, id:str):
    """
    Writes the YAML header to a file using the internal parameters
    """
    file.write("version: "+str(yaml_version)+"\n")
    file.write("frame_id: "+f_id+"\n")
    file.write("id: "+str(id)+"\n")

    return None

def print_Description(file, text: str):
    """
    Writes the mission description "text" to a file.
    """
    file.write("description: \""+text+"\"\n")

    return None

def print_Route(file, uav: UAVS.UAV, utmZone: tuple):
    """
    Writes the route of one UAV to a file
    """

    if "px4" == uav.get_Name():
        with open('./mission_px4.plan', 'w') as f:
            json.dump(px4_route_to_Plan(uav, utmZone), f, cls=NumpyArrayEncoder, indent=4)
        # return None

    file.write("  - name: \"Inspection_"+uav.get_ID()+"_"+uav.missionSettings["Base"]+"\"\n")
    file.write("    uav: \""+uav.get_ID()+"\"\n")
    file.write("    wp:\n")

    for wp in uav.waypoints:
        print(wp[0])

        latlon = CO.utm2latlon(wp[0], utmZone)
        #latlon[0], latlon[1] = latlon[1], latlon[0] 

        point_str = np.array2string(latlon, separator=", ")
        # This should flip to the correct latlon.
        if "px4" == uav.get_Name():
            file.write("      - {pos: "+point_str+", action: { command: "+str(wp[1]["command"])+"} }\n")
        else:
            file.write("      - {pos: "+point_str+", action: "+str(wp[1])+"}\n")

    file.write("    attributes:\n\n")
    file.write("      mode_landing: " + landing_Mode_to_Int(uav.missionSettings["Landing Mode"])+ "\n\n")
    file.write("      mode_yaw: "+str(mode_yaw)+ "\n\n")
    file.write("      mode_gimbal: "+str(mode_gimbal)+ "\n\n")
    file.write("      idle_vel: "+str(uav.missionSettings["Insp. speed"])+ "\n\n")
    file.write("      nav_vel: "+str(uav.missionSettings["Nav. speed"])+ "\n\n")

    return None

def print_Routes(file, uavs: UAVS.UAV_Team, utmZone: tuple):
    """
    Writes each of the UAV Team routes to a file
    """

    file.write("route:\n")

    for uav in uavs:
        if not uav.waypoints.get_Points_List(): 
            continue

        print_Route(file, uav, utmZone)
        file.write("\n\n")

    return None

def save_Dict_to_File(data: dict, file_path: str):

    if os.path.isfile(file_path):
        print("File already exists. Overwriting")

    f = open(file_path, 'w')
    yaml.dump(data, f)
    f.close()

    return None

def save_Mission(file_path, mission_id: str, uavs: UAVS.UAV_Team, utmZone: tuple):
    """
    Save the mission to a YAML file in the specified file path
    """

    if os.path.isfile(file_path):
        print("File already exists. Overwriting")

    with open(file_path, 'w') as f:

        print_Header(f, mission_id)
        print_Description(f, "")
        print_Routes(f, uavs, utmZone)

    f.close()

    print("File Saved!")

    return None

def waypoint_to_YAML(uavs: UAVS.UAV_Team) -> dict:
    """
    Outputs a dict object that contains the YAML output of the mission
    """

    wps = {}
    wps["version"] = 3
    wps["frame_id"] = "/gps"

    wps["description"] = ""

    route_list = []

    for uav in uavs:

        if not uav.waypoints.get_Points_List(): 
            continue

        route_dict = {}

        route_dict["name"] = "Inspection_"+uav.get_ID()+"_"+uav.missionSettings["Base"]
        route_dict["uav"] = uav.get_ID()

        wp_list = []

        for wp in uav.waypoints:
            wp_list.append({"pos": wp[0].tolist(), "action": str(wp[1])} )

        route_dict["wp"] = wp_list

        route_dict["attributes"] = {
                "mode_landing": 2,
                "mode_yaw":2,
                "mode_gimbal":0,
                "idle_vel": 5.0}
            
        route_list.append(route_dict)

    wps["route"] = route_list

    print(wps)

    return wps

def load_data_from_YAML(file_path: str) -> tuple[BA.Bases, TW.Towers, UAVS.UAV_Team, WT.Weather, int, dict]:
    """
    DEPRECATED. Loads the bases, towers, uavs and weather from a YAML input file.
    """

    # Initialize data
    bases = BA.Bases()
    towers = TW.Towers()
    uavs = UAVS.UAV_Team()
    weather = WT.Weather()

    # If a YAML is loaded twice, it crashed. This fixes it
    uavs.empty()

    # Open YAML stream from the file path and load it as a Python object
    f = open(file_path, "r")
    mission_init = yaml.load(f, Loader = yaml.Loader)
    f.close()

    print(mission_init)

    # Load the UAVs
    k = 0
    for uav_dict in mission_init["settings"]["devices"]:

        uav = UAVS.UAV()
        compatibleQ = uav.load_from_Model(uav_dict["devices"]["category"],
                                           str(uav_dict["devices"]["deviceId"]), mission_init["settings"]["type"])

        # If the model is not compatible with current use case.
        if not compatibleQ: 
            print(f"UAV {uav_dict['devices']['category']} is not compatible with use case {mission_init['settings']['type']}")
            continue

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
        
        uav.extra_parameters["Tower Height"] = geom["tower"]

        if 1 == mission_init["settings"]["type"]:
            uav.extra_parameters["Orbital Points"] = 5 # This is supossed to come in the json or yaml file]

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

    # This dict handle extra parameters that might be mode dependant.
    # This is then loaded into the problem with **kwargs as Parameters
    mode_parameters = {}

    return bases, towers, uavs, weather, mission_init["settings"]["type"], mode_parameters

def load_data_from_JSON(json_obj) -> tuple[BA.Bases, TW.Towers, UAVS.UAV_Team, WT.Weather, int, str, dict]:
    """
    Loads the bases, towers, uavs and weather from a JSON object.

    Returns bases, towers, uavs, weather, misssion_mode, mission_id
    """

    # Initialize data
    bases = BA.Bases()
    towers = TW.Towers()
    uavs = UAVS.UAV_Team()
    weather = WT.Weather()

    # If a YAML is loaded twice, it crashed. This fixes it
    uavs.empty()

    # Load the UAVs
    k = 0
    for uav_dict in json_obj["devices"]:

        uav = UAVS.UAV()
        compatibleQ = uav.load_from_Model(uav_dict["category"], str(uav_dict["id"]), json_obj["case"])

        # If the model is not compatible with current use case.
        if not compatibleQ: 
            print(f"UAV {uav_dict['category']} is not compatible with use case {json_obj['case']}")
            continue

        uav.missionSettings["Base"] = "B"+str(k)
        uav.missionSettings["Nav. speed"] = uav_dict["settings"]["navigation_speed"]
        uav.missionSettings["Insp. speed"] = uav_dict["settings"]["inspection_speed"]
        uav.missionSettings["Landing Mode"] = int_to_Landing_Mode(uav_dict["settings"]["landing_mode"])
        uav.missionSettings["Insp. height"] = uav_dict["settings"]["vertical_offset"]
        uav.missionSettings["Insp. horizontal offset"] = uav_dict["settings"]["horizontal_offset"]

        # Update from the other pair
        uav.missionSettings['Tower distance'] = float(np.sqrt(
            uav.missionSettings['Insp. height']**2 + uav.missionSettings['Insp. horizontal offset']**2))
        
        temp = uav.missionSettings['Insp. horizontal offset']
        if temp == 0:
            uav.missionSettings['Cam. angle'] = 90
        else:
            uav.missionSettings['Cam. angle'] = float(np.rad2deg(np.arctan(
                uav.missionSettings['Insp. height'] / temp)))
            
        uav.extra_parameters["Tower Height"] = uav_dict["settings"]["tower_height"]

        try:
            uav.extra_parameters["Take Off Height"] = uav_dict["settings"]["takeoff_height"]
        except:
            None

        # PX4 Exclusives
    
        try:
            uav.extra_parameters["Max. Angle"] = uav_dict["settings"]["max_angle"]
        except:
            None
        try:
            uav.extra_parameters["Min. Radius"] = uav_dict["settings"]["min_radius"]
        except:
            None

        if 1 == json_obj["case"]:
            uav.extra_parameters["Orbital Points"] = uav_dict["settings"]["orbital_points"]
            uav.extra_parameters["Security Height"] = uav_dict["settings"]["security_height"]

        uavs.add_UAV(copy.deepcopy(uav))

        coords = np.array([uav_dict["settings"]["base"][0], uav_dict["settings"]["base"][1], 0])
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

    for loc in json_obj["locations"]:
        firstQ = True
        for point in loc["items"]:

            if True == firstQ:
                path = np.array([[point["latitude"], point["longitude"], 0.0]])
                firstQ = False
            else: path = np.concatenate((path, np.array([[point["latitude"], point["longitude"], 0.0]])))

        paths.append(path)

    print(paths)

    towers.load_from_Arrays(paths, True)

    # This dict handle extra parameters that might be mode dependant.
    # This is then loaded into the problem with **kwargs as Parameters
    mode_parameters = {}

    return bases, towers, uavs, weather, json_obj["case"], json_obj["id"], mode_parameters

# This still uses the generic waypoint system to allow the implementation of YAMLs 
# for px4 in the future

def px4_route_to_Plan(uav: UAVS.UAV, utmZone: tuple) -> dict:
    """
    Transform a regular px4 route into a .plan-compatible format.

    Ref: https://docs.qgroundcontrol.com/master/en/qgc-dev-guide/file_formats/plan.html
    """

    # First check if the category is correct
    if "px4" != uav.get_Name():
        print("px4_route_to_Plan: This UAV is not a valid px4 or DeltaQuad. Ignoring")
        return None

    # Fixed data within the .plan
    plan = {
        "fileType": "Plan",                  # Must be "Plan" always
        "groundStation": "smart-flies@GRVC", # Can be changed
        "version": 1,                        # Current version
        "geoFence": {                        # Leave as it is
            "circles": [],
            "polygons": [],
            "version": 2
        },
        "rallyPoints": {                     # Leave as it is
            "points": [],
            "version": 2
        },
    }

    base_latlon = CO.utm2latlon(uav.routeUTM[0][0], utmZone)

    # Fixed data within the mission item of the .plan
    mission = {
        "version": 2,                                      # Current version
        "firmwareType": 12,                                # 12 refers to the PX4 Autopilot firmware
        "globalPlanAltitudeMode": 1,                       # This is the default reference mode for height
                                                           # for points without "AltitudeMode".
                                                           # 1 -> Altitude respect to base; 0 -> Altitude respect to sea level
        "vehicleType": 20,                                 # 20 = MAV_TYPE_VTOL_TAILSITTER_QUADROTOR
        "cruiseSpeed": uav.missionSettings["Nav. speed"],  # Ask about speeds
        "hoverSpeed": 5,
        "plannedHomePosition": [                           # Base coordinates 
            base_latlon[0],
            base_latlon[1],
            0 
        ]
    }
    item_list = []

    # This suposses that the waypoints are already valid for the px4 physical limitations
    k = 1
    for point, actions, mode in uav.waypoints:
        # wp = (point_vector, actions_dict, mode_str)

        if "DEPRECATED" == mode: # Complex_Item
            item = {
                "altitudesAreRelative": True,
                "complexItemType": "vtolLandingPattern",
                "landCoordinate": actions["Landing Point"],
                "landingApproachCoordinate": actions["Approach Point"],
                "loiterClockwise": actions["Loiter Clockwise"],
                "loiterRadius": actions["Loiter Radius"],
                "stopTakingPhotos": True,
                "stopVideoPhotos": True,
                "type": "ComplexItem",
                "useLoiterToAlt": True,
                "version": 1
            }
        else:                 # Simple_Item
            item = {
                "type": "SimpleItem",
                "AMSLAltAboveTerrain": None,
                "AltitudeMode": 1,
                "Altitude": point[2],
                "autoContinue": True,
                "command": actions["command"], # 16 for waypoints, 84 for Take off
                "doJumpId": k,
                "frame": 3,                    # For relative global altitude
                "params": actions["params"]    # Params depend on the command (MAV_CMD)
            }
            k += 1

        item_list.append(item)
           
    # Add waypoints item to the mission and it to the plan
    mission["items"] = item_list
    plan["mission"] = mission

    return plan