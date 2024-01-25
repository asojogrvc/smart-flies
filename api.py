# General imports

from flask import Flask, request, jsonify, send_from_directory, flash, redirect, render_template, url_for
import os

# Projects internal modules imports
from modules import bases as BA, towers as TW, uav as UAVS, solver as SO, weather as WT, coordinates as CO, yaml as iYAML

# --------------------------------------------------------------------
#
#    Derive the Flask class to allow new parameters and methods
#
# --------------------------------------------------------------------


class Server(Flask):
    """
    This is a derived class from Flask. It contains all the extra data that needs to be accessed for the
    I/O to happen interactively.
    """
    __status = "inactive"    # By default, the API server is inactive.
    __bases_Input = "file"   # How was data received
    __towers_Input  = "file" # How was data received

    def set_Status(self, statusI: str):

        allowed_states = ["inactive", "occupied"]

        if statusI in allowed_states:
            self.__status = statusI
            return None
        
        else:
            raise Exception("Tried to change the server status to an invalid one.")

    def get_Status(self) -> str:
        return self.__status
    
    def set_Input_type(self, new_type: str, which:str):

        valid_types = ["file", "json", "yaml"]

        if new_type in valid_types:

            match which:
                case "Bases":
                    self.__bases_Input = new_type
                case "Towers":
                    self.__towers_Input = new_type
                case _:
                    raise Exception("Not a valid input parameter")
                
        else:
            raise Exception("New type is not valid")
        
        return None
    
    def get_Input_type(self, which:str) -> str:

        match which:
            case "Bases":
                return self.__bases_Input
            case "Towers":
                return self.__towers_Input
            case _:
                raise Exception("Not a valid input parameter")

template_dir = os.path.abspath('./server/static/')
app = Server(__name__, template_folder=template_dir, static_url_path="", static_folder="./server/static/")

# --------------------------------------------------------------------
#
#                        Define the API routes
#
# --------------------------------------------------------------------


# --------------------- Data input Routes ----------------------------
# https://sentry.io/answers/flask-getting-post-data/
# https://flask.palletsprojects.com/en/2.3.x/patterns/fileuploads/

@app.route('/input_yaml', methods = ['POST'])
def receive_YAML() -> dict | None:
    """
    Receive all the needed data to define the problem from an YAML file.
    """
    # Check if the request contains a file.

    # If not, consider it a data stream for some content type
    if 'file' not in request.files:

        return {"output" : "Not a file"}
    
    # If it does, treat the data as a file.
    else:

        file = request.files['file']
        # If the user does not select a file, the browser submits an
        # empty file without a filename.
        if file.filename == '':
            return {"output": "Empty File"}
        if file:

            file.save("./server/dynamic/mission_init.yaml")

            app.set_Input_type("yaml", "Bases")
    
            return {"output": "YAML Received"}
        

@app.route('/input_bases', methods = ['POST'])
def receive_bases() -> dict | None:
    """
    Receive the needed data to define bases. It can be done as an "application/json" or
    as an .kml file.
    """

    # Check if the request contains a file.

    # If not, consider it a data stream for some content type
    if 'file' not in request.files:

        content_type = request.headers.get('Content-Type')

        if (content_type == 'application/json'):

            # request.get_json()

            app.set_Input_type("json", "Bases")
            return {"output":"JSON Received"}
        else:
            return {"output":"No file in the request and the stream is not a json"}
    
    # If it does, treat the data as a file.
    else:

        file = request.files['file']
        # If the user does not select a file, the browser submits an
        # empty file without a filename.
        if file.filename == '':
            return {"output": "No file or json in request"}
        if file:

            file.save("./server/dynamic/bases.kml")

            app.set_Input_type("file", "Bases")
            return {"output": "KML Received"}

@app.route('/input_towers', methods = ['POST'])
def receive_towers() -> dict | None:
    """
    Receive the needed data to define the towers. It can be done as an "application/json" or
    as an .kml file.
    """

    # Check if the request contains a file.

    # If not, consider it a data stream for some content type
    if 'file' not in request.files:

        content_type = request.headers.get('Content-Type')

        if (content_type == 'application/json'):
            
            # request.get_json()

            app.set_Input_type("json", "Towers")
            return {"output":"JSON Received"}
        else:
            return {"output":"No file in the request and the stream is not a json"}
    
    # If it does, treat the data as a file.
    else:

        file = request.files['file']
        # If the user does not select a file, the browser submits an
        # empty file without a filename.
        if file.filename == '':
            return {"output": "No file or json in request"}
        if file:

            file.save("./server/dynamic/towers.kml")

            app.set_Input_type("file", "Towers")
            return {"output": "KML Received"}

@app.route('/favicon.ico', methods = ["GET"])
def favicon():
    """
    Webpage icon for web browsers.
    """
    return send_from_directory(os.path.join(app.root_path, 'server','static'),
                          'favicon.ico',mimetype='image/vnd.microsoft.icon')

@app.route("/", methods = ["GET"])
def main_page():
    """
    Main Page. Put some logos, copyright and stuff here.
    """
    return render_template("main.html", server_status = app.get_Status())

@app.route("/status", methods = ["GET"])
def status():
    """
    Outputs the current status of the server
    """

    return {"status": app.get_Status()}

@app.route("/output", methods = ["GET"])
def yaml_output():
    """
    Outputs the file last file mission if it exists
    """
    return send_from_directory("./server/dynamic/", 'mission.yaml')

@app.route("/mission_request", methods = ["GET"])
def planner():

    # If the server is already doing some computations,
    # do not execute the planner again.

    if "Occupied" == app.get_Status():
        return {"status": "Occupied"}
    
    app.set_Status("occupied")
    
    # --- Placeholder data -------------------------

    if "yaml" == app.get_Input_type("Bases"):
        bases, towers, uavs, weather = iYAML.load_data_from_YAML("./server/dynamic/mission_init.yaml")

    else:

        bases = BA.Bases()
        if "file" == app.get_Input_type("Bases"): bases.load_File("./server/dynamic/bases.kml", True)
        else: bases.load_File("./files/bases.kml", True) # TBI

        towers = TW.Towers()
        if "file" == app.get_Input_type("Towers"): towers.load_File("./server/dynamic/towers.kml", True)
        else: towers.load_File("./files/towers.kml", True) # TBI

        uavs = UAVS.UAV_Team()
        uavs.load_File("./files/UAV_team.xml")
    

        weather = WT.Weather()
        base1 = bases.get_Base("B1")
        weather.update_Online(CO.utm2latlon(base1.get_Coordinates(), base1.get_UTM_Zone()))
    
        
        # ----------------------------------------------

    problem = SO.Problem(towers, bases, uavs, weather)

    problem.solve("abstract")

    problem.get_UAV_Team().compute_Team_Waypoints("PaV", problem.get_Towers(), problem.get_Bases())
    base0 = problem.get_Bases().get_Base("B0")
    iYAML.save_Mission("./server/dynamic/mission.yaml", problem.get_UAV_Team(), base0.get_UTM_Zone())
        
    app.set_Status("inactive")
        
    return {"output" : "The problem has been solved successfully. Updated file from /output"}