# General imports

from flask import Flask, request, jsonify, send_from_directory
import os
import yaml

# Projects internal modules imports
from modules import bases as BA, towers as TW, uav as UAVS, solver as SO, weather as WT, coordinates as CO, yaml as iYAML


class Server(Flask):
    """
    This is a derived class from Flask. It contains all the extra data that needs to be accessed for the
    I/O to happen interactively.
    """
    __status = "Inactive"

    def set_Status(self, statusI):
        self.__status = statusI
        return None

    def get_Status(self) -> str:
        return self.__status

app = Server(__name__)

# ------------------------ Define the api routes ---------------------

@app.route('/favicon.ico')
def favicon():
    """
    Webpage icon for web browsers.
    """
    return send_from_directory(os.path.join(app.root_path, 'server','static'),
                          'favicon.ico',mimetype='image/vnd.microsoft.icon')

@app.route("/")
def main_page():
    """
    Main Page. Put some logos, copyright and stuff here.
    """
    return "</p>If you can see this, the server is online</p>"

@app.route("/status")
def status():
    """
    Outputs the current status of the server
    """

    return {"status": app.get_Status()}

@app.route("/output")
def yaml_output():
    """
    Outputs the file last file mission if it exists
    """

    return send_from_directory(os.path.join(app.root_path), 'mission.yaml')

@app.route("/mission_request/")
def planner():

    # If the server is already doing some computations,
    # do not execute the planner again.

    if "Occupied" == app.get_Status():
        return {"status": "Occupied"}
    
    computeQ = eval(request.args.get('computeQ'))
    
    if computeQ == True:
        app.set_Status("Occupied")
    
        # Placeholder data
        bases = BA.Bases()
        bases.load_File("./files/bases.kml", True)

        towers = TW.Towers()
        towers.load_File("./files/towers.kml", True)

        uavs = UAVS.UAV_Team()
        uavs.load_File("./files/UAV_team.xml")

        weather = WT.Weather()
        base1 = bases.get_Base("B1")
        weather.update_Online(CO.utm2latlon(base1.get_Coordinates(), base1.get_UTM_Zone()))

        problem = SO.Problem(towers, bases, uavs, weather)
        problem.solve("abstract")

        uavs.compute_Team_Waypoints("PaV", towers, bases)
        yaml_out = iYAML.waypoint_to_YAML(uavs)

        app.set_Status("Inactive")
        iYAML.save_Mission("./mission.yaml", uavs, base1.get_UTM_Zone())
        return "</p>Done</p>"
    
    else:
        app.set_Status("Inactive")
        return "</p>Ayo mate, you forgot the ?computeQ=True</p>"