from os import stat
import requests
import os
from requests.models import stream_decode_response_unicode
import websockets
import asyncio
import json
import pathlib

CONTROL_HUB_URL = "http://localhost:8080"
CONTROL_HUB_WSURL = "ws://localhost:8081"
CONTROL_HUB_REFERER = CONTROL_HUB_URL + "/java/editor.html"
CONTROL_HUB_UPLOAD_URL = CONTROL_HUB_URL + "/java/file/upload"
SRC_DIRECTORY = pathlib.Path("TeamCode/src/main/java/org/firstinspires/ftc/teamcode")

def uploadJavaFile(path: str):
    if path.endswith(".java"):
        files = {'file': (os.path.basename(path), open(path, 'rb'), 'application/octet-stream')}
        return requests.post(CONTROL_HUB_UPLOAD_URL, files=files)
        

async def build():
    async with websockets.connect(CONTROL_HUB_WSURL) as ws:
        # Subscribe to OnBotJava updates
        await ws.send('{"namespace":"system","type":"subscribeToNamespace","payload":"ONBOTJAVA"}')
        
        # Send build request
        await ws.send('{"namespace":"ONBOTJAVA","type":"build:launch","payload":""}')
        status = ""
        
        while status != "SUCCESSFUL" and status != "FAILED":
            msg = await ws.recv()
            j = json.loads(msg)
            payload = json.loads(j['payload'])
            status = payload['status']
            print (f"Build Status: {status}")
        
for path, subdirs, files in os.walk(SRC_DIRECTORY):
    if (not "TeamCode/src/main/java/org/firstinspires/ftc/teamcode/virtual" in path):
        for name in files:
            file_path = os.path.join(path, name)
            r = uploadJavaFile(file_path)
            if (r is not None and r.status_code == 200):
                print (f"Uploaded: {r.text}")

print ("Upload comlete, building...")
asyncio.get_event_loop().run_until_complete(build())
