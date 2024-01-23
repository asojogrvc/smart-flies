from flask import Flask, request, jsonify
import yaml

app = Flask(__name__)

@app.route("/")
def main_page():
    return "</p>Hola</p>"

@app.route("/mission_request/")
def yaml_test():

    computeQ = eval(request.args.get('computeQ'))

    data = jsonify(None)
    if computeQ:
        f = open("mission.yaml")
        data = yaml.safe_load(f)
        f.close()

    return data


#if __name__ == '__main__':
#    app.run(debug=False, host = '0.0.0.0', port=8001)
