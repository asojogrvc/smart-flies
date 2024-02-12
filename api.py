# General imports

from flask import Flask, request, jsonify, send_from_directory, flash, redirect, render_template, url_for
from flask_cors import CORS
import os, glob, json, yaml, threading

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
# threaded=False, processes=3

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

@app.route('/uav_database', methods = ["GET"])
def uav_database() -> str:

    f = open("./files/devices.yaml", "r")
    database = yaml.load(f, Loader = yaml.Loader)
    f.close()

    html_data = """
        <!DOCTYPE html>
        <html lang="en">
        <head>
            <meta charset="UTF-8">
            <title>Smart Flies Project</title>
        </head>
        <link rel="icon" href="
        """+url_for('static', filename='favicon.ico')+"""" />
        <style>
            table, th, td {
                border:1px solid black;
            }
        </style>
        <body>
            <center><img src="
        """ + url_for('static', filename='ico.png') + """
                " alt="Project logo" width="50" height="50">
            <h1>Smart Flies Project</h1></center>
   
            <table style="width:100%">
                <tr>
                    <th>Model</th>
                    <th>Mass [kg]</th>
                    <th>Number of rotors</th>
                    <th>Blades per rotor</th>
                    <th>Rotor radius [m]</th>
                    <th>Blade chord [m]</th>
                    <th>Lift Coefficient</th>
                    <th>Drag Coefficient</th>
                    <th>Induced Power Factor</th>
                    <th>Energy Efficiency</th>
                    <th>Equivalent Flat Plate Area</th>
                    <th>Battery</th>
                </tr>
        """
    
    for model in database:
            
            data = database[model]
            model_column = f"""
            <tr>
                <th>{model}</th>
                <th>{data['mass']}</th>
                <th>{data['number_of_rotors']}</th>
                <th>{data['rotor_blades']}</th>
                <th>{data['rotor_radius']}</th>
                <th>{data['blade_chord']}</th>
                <th>{data['lift_coefficient']}</th>
                <th>{data['draft_coefficient']}</th>
                <th>{data['induced_power_factor']}</th>
                <th>{data['energy_efficiency']}</th>
                <th>{data['P0_numerical_constant']}</th>
                <th>{data['equivalent_flat_plate_area']}</th>
            <tr>
            """
            html_data = html_data + model_column

    html_data = html_data + """
            </table> 

            <center><p align = bottom>&copy; Antonio Sojo@GRVC</p></center>
        </body>
        </html>
        """

    return html_data

@app.route('/mission_request', methods = ['POST'])
def mission_Request() -> dict | None:
    """
    Receives a JSON, start the planification and saves the plan into a local file.
    """
    try:
        mission =  request.get_json()
    except:
        output = {"output": "Not a JSON"}

    thread = threading.Thread(target=planner, kwargs={'mission_json': mission})
    thread.start()

    return {"output": "JSON Received. Computing..."}

def planner(mission_json):
    """
    Takes a mission as a json, computes the plan and saves it to a local file with its ID
    """

    # If the server is already doing some computations,
    # do not execute the planner again. I don't think this is necessary as it runs single-threaded

    if "occupied" == app.get_Status():
        return {"status": "occupied"}
    
    app.set_Status("occupied")

    print(mission_json)

    try: 
        bases, towers, uavs, weather, mode, id = iYAML.load_data_from_JSON(mission_json)

        problem = SO.Problem(towers, bases, uavs, weather, mode)

        status = problem.solve("")

        if False == status:
            app.set_Status("inactive")
            iYAML.save_Dict_to_File({str(id): "Planner failed: infeasibility"}, "./server/dynamic/mission_"+str(id)+".yaml")

        problem.get_UAV_Team().compute_Team_Waypoints(problem.get_Mission_Mode(), problem.get_Towers(), problem.get_Bases())
        base0 = problem.get_Bases().get_Base("B0")
        iYAML.save_Mission("./server/dynamic/mission_"+str(id)+".yaml", problem.get_UAV_Team(), base0.get_UTM_Zone())
        app.set_Status("inactive")

    except:

        app.set_Status("inactive")
        iYAML.save_Dict_to_File({str(id): "JSON Format not valid or the planner failed"}, "./server/dynamic/mission_"+str(id)+".yaml")

    return None


@app.route("/get_plan", methods = ["GET"])
def json_output():
    """
    Outputs a json of the requested missions (by IDs) if all of them exist
    """

    # here we want to get the value of ids (i.e. ?IDs=1,2,3,)
    ids = request.args.get('IDs')

    if "" == ids:
        return {"output": "No IDs given"}
    
    try:
        ids = ids.split(",")
    except:
        return {"output": "No IDs syntax"}

    json = {}
    for id in ids:

        try:
            f = open("./server/dynamic/mission_"+id+".yaml", 'r')
            mission_data = yaml.safe_load(f)
        except:
            mission_data = "Not a valid ID"

        json[id] = mission_data

    return jsonify(json)