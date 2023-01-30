from logging import debug
from core import app, socketio
import time
from random import randrange
import sys
import signal
sys.path.append('../buildVolumeScanner')
# sys.path.append('../buildTest')
import numpy as np
from PIL import Image  
import cv2
import base64

import cameraBindings
cameraBindings.readconfig("../buildVolumeScanner/config.txt")
# camtype = cameraBindings.CameraTypes.IntelCamera
# camtype = cameraBindings.CameraTypes.AzureKinect
camtype = cameraBindings.CameraTypes.EnsensoCamera
# camtype = cameraBindings.CameraTypes.ZividCamera
n_cams = cameraBindings.getNumberofConnectedDevices(camtype) #todo we must call this for init atm
cam = cameraBindings.CameraWrapper(camtype)
cam.connect(0)
cam.ReadFOVDataFromDisk("/home/tristan/dev/StaticScanner_cpp/resources/config/")


overlayImageEnabled = True

@socketio.on("receiveOverlayToogle")
def receiveOverlayToogleFunction(data):
    global overlayImageEnabled
    overlayImageEnabled = data["overlayToogle"]




def cv2base64(img):
    img = np.array(img)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    height, width, channels = img.shape
    success, compressed_frame = cv2.imencode('.jpg', img, [int(cv2.IMWRITE_JPEG_QUALITY), 75])
    if success:
        content = compressed_frame.tobytes()
        jpg_as_text = base64.b64encode(content)
        return height, width, jpg_as_text
    else:
        print("Encoding failed")
        return False

stopThread =False
thread = None
def thread_function():
    global stopThread
    while not stopThread:
        socketio.sleep(1.2)

        cam.clearBuffers()
        success = cam.record(10,0)
        cam.AlignUndistortConvert()
        cam.CropRGBDImages()
        if not success:
            print("recording failed, starting anew")
            continue
        
        socketio.emit('neuesVolumen', {'datensatz': 1000*cameraBindings.getCurrentVolume(cam)}, broacast=True)
        if overlayImageEnabled:
            height2, width2, img2 = cv2base64(cam.VolumeImage)
            socketio.emit("newCameraImage", {"payload": img2, "width": width2, "height": height2})
        else:
            height2, width2, img2 = cv2base64(cam.FRCalibrationImages[0])
            socketio.emit("newCameraImage", {"payload": img2, "width": width2, "height": height2})

    print("Closing thread")



def signal_handler(signal, frame):
    global stopThread
    print("exiting")
    stopThread = True
    socketio.stop()
    thread.join()
    cam.disconnect()
    sys.exit()

#   sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)


if __name__ == "__main__":
    thread = socketio.start_background_task(thread_function)
    socketio.run(app, host='192.168.2.104', port=8080)  # lock at debugger options
    # cam.record(1)
    # cam.AlignUndistortConvert()
    # a = cam.pythonImage
    # img = np.array(a)
    # img = img/255
    # cv2.imshow("test",img)
    # cv2.waitKey(0)
    # a = np.array(cam.pythonImage)19
    # print(a)


