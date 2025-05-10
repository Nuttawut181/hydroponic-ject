import os
from supabase import create_client, Client
from flask import Flask
from dotenv import load_dotenv

load_dotenv()

url: str = os.getenv("SUPABASE_URL")
key: str = os.getenv("SUPABASE_TOKEN")
supabase: Client = create_client(url, key)

app = Flask(__name__)

@app.route("/")
def hello_world():
    response = (
        supabase.table("hydroponic sensor")
        .select("*")
        .execute()
    )
    return response.model_dump()

@app.route("/<light>/<temp>/<humidity>/<tds>")
def addSensorData(light, temp, humidity, tds):
    response = (
        supabase.table("hydroponic sensor")
        .insert({
            "light": light,
            "temperature": temp,
            "humidity": humidity,
            "tds": tds,
        })
        .execute()
    )
    return response.data