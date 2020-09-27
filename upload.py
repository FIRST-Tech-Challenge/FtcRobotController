import requests
import os
import websockets
import asyncio

CONTROL_HUB_URL = "http://localhost:8080"
CONTROL_HUB_WSURL = "ws://localhost:8081"
CONTROL_HUB_REFERER = CONTROL_HUB_URL + "/java/editor.html"
CONTROL_HUB_UPLOAD_URL = CONTROL_HUB_URL + "/java/file/upload"
SRC_DIRECTORY = "C:\\Users\\lvern\\Documents\\GitHub\\UltimateGoal\\TeamCode\\src\\main\\java\\org\\firstinspires\\ftc\\teamcode"

def uploadJavaFile(path: str):
    if path.endswith(".java"):
        files = {'file': (os.path.basename(path), open(path, 'rb'), 'application/octet-stream')}
        r = requests.post(CONTROL_HUB_UPLOAD_URL, files=files)
        print (r.text)

async def build():
    async with websockets.connect(CONTROL_HUB_WSURL) as ws:
        await ws.send('{"namespace":"ONBOTJAVA","type":"build:launch","encodedPayload":""}')

for path, subdirs, files in os.walk(SRC_DIRECTORY):
    for name in files:
        file_path = os.path.join(path, name)
        uploadJavaFile(file_path)

asyncio.get_event_loop().run_until_complete(build())