# General imports

from flask import Flask, request, jsonify, send_from_directory, flash, redirect, render_template, url_for
from flask_cors import CORS
import json
import os, glob, yaml

# Projects internal modules imports
from modules import bases as BA, towers as TW, uav as UAVS, solver as SO, weather as WT, coordinates as CO, yaml as iYAML

def delete_all_mission_data():
    files = glob.glob('./server/dynamic/*.yaml')
    for f in files:
        os.remove(f)

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

    delete_all_mission_data()

    def set_Status(self, statusI: str):

        allowed_states = ["inactive", "occupied"]

        if statusI in allowed_states:
            self.__status = statusI
            return None
        
        else:
            raise Exception("Tried to change the server status to an invalid one.")

    def get_Status(self) -> str:
        return self.__status


template_dir = os.path.abspath('./server/static/')
app = Server(__name__, template_folder=template_dir, static_url_path="", static_folder="./server/static/")

# To allow CORS External Origins
cors = CORS(app, resources={r"/*": {"origins": "*"}})

# --------------------------------------------------------------------
#
#                        Define the API routes
#
# --------------------------------------------------------------------


# --------------------- Data input Routes ----------------------------
# https://sentry.io/answers/flask-getting-post-data/
# https://flask.palletsprojects.com/en/2.3.x/patterns/fileuploads/


@app.route('/favicon.ico', methods = ["GET"])
def favicon():
    """
    Webpage icon for web browsers.
    """
    return send_from_directory(os.path.join(app.root_path, 'server','static'),
                          'favicon.ico', mimetype='image/vnd.microsoft.icon')

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


@app.route('/mission_request', methods = ['POST'])
def mission_Request() -> dict | None:
    """
    Receives a JSON, start the planification and saves the plan into a local file.
    """

    try:
        output = planner(request.get_json())
    except:
        output = {"output" : "Something went wrong"}

    return output

def planner(mission_json):

    # If the server is already doing some computations,
    # do not execute the planner again.

    if "Occupied" == app.get_Status():
        return {"status": "Occupied"}
    
    app.set_Status("occupied")

    bases, towers, uavs, weather, mode, id = iYAML.load_data_from_JSON(mission_json)

    problem = SO.Problem(towers, bases, uavs, weather, mode)

    status = problem.solve("")

    if False == status:
        return {"output" : "The problem is infeasible"}

    problem.get_UAV_Team().compute_Team_Waypoints(problem.get_Mission_Mode(), problem.get_Towers(), problem.get_Bases())
    base0 = problem.get_Bases().get_Base("B0")
    iYAML.save_Mission("./server/dynamic/mission_"+str(id)+".yaml", problem.get_UAV_Team(), base0.get_UTM_Zone())
        
    app.set_Status("inactive")
        
    return {"output" : "The problem has been solved successfully. Updated file to /get_plan"}


@app.route("/get_plan", methods = ["GET"])
def json_output():
    """
    Outputs a json of the requested missions (by IDs) if all of them exist
    """

    # here we want to get the value of user (i.e. ?IDs=1,2,3,)
    ids = request.args.get('IDs')
    ids = ids.split(",")

    json = {}
    for id in ids:

        try:
            f = open("./server/dynamic/mission_"+id+".yaml", 'r')
            mission_output = yaml.safe_load(f)
        except:
            return {"output": "Some of the requested IDs are not valid or do not exist"}

        json[id] = mission_output

    return jsonify(json)