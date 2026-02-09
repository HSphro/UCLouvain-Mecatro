from picamera2 import Picamera2
import cv2
import cv2.aruco as aruco
import time

# Camera config
cam = Picamera2()
config = cam.create_preview_configuration(
    main={"size": (640, 480), "format": "RGB888"}
)
cam.configure(config)
cam.start()
time.sleep(0.5)

# ArUco setup
ARUCO_DICT = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
parameters = aruco.DetectorParameters_create()

while True:
    frame = cam.capture_array()
    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

    corners, ids, rejected = aruco.detectMarkers(
        gray, ARUCO_DICT, parameters=parameters
    )

    if ids is not None:
        for marker_id in ids.flatten():
            print(f"Detected ArUco ID: {marker_id}")



    
