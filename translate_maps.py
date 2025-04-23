import json
import os
import numpy as np

from modules.coordinates import epsg30352latlon

def translate_file(file_path):

    pos_offset = np.array([2075729.477689, 3258375.385138])

    name = os.path.basename(file_path).split('/')[-1].split('.')[0]

    old_data = {}

    with open(file_path) as f:
        new_data = json.load(f)

    old_data["id"] = "33"
    old_data["name"] = name
    old_data["case"] = 0
    old_data["meteo"] = []
    old_data["devices"] = [
        {
            "settings": {
                "base": epsg30352latlon(np.array(new_data["Bases"]["B"][:2]) + pos_offset).tolist(),
                "inspection_speed": 7,
                "navigation_speed": 15,
                "vertical_offset": 10,
                "horizontal_offset": 10,
                "tower_height": 10,
                "security_height": 10,
                "orbital_points": 6,
                "landing_mode": 2
            },
            "id": "uav_0",
            "category": "dji_M210_noetic"
        },
    ]
    old_data["location"] = [
        {
            "name": "Elements",
            "items":[
                {
                    "latitude": epsg30352latlon(np.array(tower[:2])+pos_offset)[0],
                    "longitude": epsg30352latlon(np.array(tower[:2])+pos_offset)[1]
                }
                for tower in new_data["Towers"]["List"].values()
            ]
        }
    ]


    with open("./files/translated/"+name+"_t.json", 'w+') as fp:
        json.dump(old_data, fp, indent=2)

file_path = "./files/new_format/eil22.json"
translate_file(file_path)