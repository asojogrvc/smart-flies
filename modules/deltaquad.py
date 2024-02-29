# PX4 / DeltaQuad Code

import uav as UAVS
import numpy as np

def px4_route_to_Plan(uav: UAVS.UAV):
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

    # Fixed data within the mission item of the .plan
    mission = {
        "version": 2,                                      # Current version
        "firmwareType": 12,                                # 12 refers to the PX4 Autopilot firmware
        "globalPlanAltitudeMode": 0,                       # This is the default reference mode for height
                                                           # for points without "AltitudeMode".
                                                           # 0 -> Altitude respect to base; 1 -> Altitude respect to sea level
        "vehicleType": 20,                                 # 20 = MAV_TYPE_VTOL_TAILSITTER_QUADROTOR
        "cruiseSpeed": uav.missionSettings["Nav. speed"],  # Ask about speeds
        "hoverSpeed": 5,
        "plannedHomePosition": [                           # Base coordinates 
            uav.routeUTM[0][0],
            uav.routeUTM[0][1],
            0 
        ]
    }
    item_list = []

    # This suposses that the waypoints are already valid for the px4 physical limitations
    for point, actions, mode in uav.waypoints:
        # wp = (point_vector, actions_dict, mode_str)

        simple_item = {
            "type": "SimpleItem",
            "AMSLAltAboveTerrain": "null",
            "Altitude": point[2],
            "autoContinue": True,
            "command": 16,  # 16 FOR WAYPOINTS,  
        }

        None                         

    # Add waypoints item to the mission and it to the plan
    mission["items"] = item_list
    plan["mission"] = mission
