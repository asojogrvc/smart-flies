# General imports

from flask import Flask, request, jsonify, send_from_directory, flash, redirect
import os

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

@app.route('/input_bases', methods=['POST'])
def receive_bases():
    """
    https://sentry.io/answers/flask-getting-post-data/
    """

    content_type = request.headers.get('Content-Type')

    if (content_type == 'application/json'):
        return {"output":"Content type is not supported."}
    else:
        return {"output":"Content type is not supported."}
    
@app.route('/input_towers', methods = ['GET', 'POST'])
def receive_towers():
    """
    https://flask.palletsprojects.com/en/2.3.x/patterns/fileuploads/
    """
    if request.method == 'POST':
        # check if the post request has the file part
        if 'file' not in request.files:
            flash('No file part')
            return redirect(request.url)
        
        file = request.files['file']
        # If the user does not select a file, the browser submits an
        # empty file without a filename.
        if file.filename == '':
            flash('No selected file')
            return redirect(request.url)
        if file and allowed_file(file.filename):
            filename = secure_filename(file.filename)
            file.save(os.path.join(app.config['UPLOAD_FOLDER'], filename))
            return redirect(url_for('download_file', name=filename))



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
    return send_from_directory("./server/", 'mission.yaml')

@app.route("/mission_request")
def planner():

    # If the server is already doing some computations,
    # do not execute the planner again.

    if "Occupied" == app.get_Status():
        return {"status": "Occupied"}
    
    app.set_Status("Occupied")
    
    # --- Placeholder data -------------------------
    bases = BA.Bases()
    bases.load_File("./files/bases.kml", True)

    towers = TW.Towers()
    towers.load_File("./files/towers.kml", True)

    uavs = UAVS.UAV_Team()
    uavs.load_File("./files/UAV_team.xml")
    # ----------------------------------------------

    weather = WT.Weather()
    base1 = bases.get_Base("B1")
    weather.update_Online(CO.utm2latlon(base1.get_Coordinates(), base1.get_UTM_Zone()))

    problem = SO.Problem(towers, bases, uavs, weather)
    problem.solve("abstract")

    uavs.compute_Team_Waypoints("PaV", towers, bases)
    iYAML.save_Mission("./server/mission.yaml", uavs, base1.get_UTM_Zone())
        
    app.set_Status("Inactive")
        
    return {"result" : "The problem has been solved successfully. Updated file from /yaml_output"}