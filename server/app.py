from flask import Flask

app = Flask(__name__)

@app.route("/")
def hello_world():
    return "<p>Hello, World!</p>"

@app.route("/<light>/<temp>/<tds>")
def get_light(light,temp,tds):
    return f"Light:{light} lux. {temp} c {tds} ppm"
