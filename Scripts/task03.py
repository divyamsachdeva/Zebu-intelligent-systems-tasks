#!/usr/bin/python

from pymavlink import mavutil
import time
from functions import*
import numpy as np
import cv2
from cv2 import aruco



cap = cv2.VideoCapture(0)

vehicle = mavutil.mavlink_connection('udpin:localhost:14551')
vehicle.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" %
      (vehicle.target_system, vehicle.target_component))


mode = 'GUIDED'

change_mode(vehicle, mode)

""" Mission starts here """

arm(vehicle)

takeoff_land(vehicle, "takeoff", 7)

time.sleep(3)

goto_position_target_local_ned(vehicle, 4, 6, -7)

time.sleep(5)

while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()

    # Our operations on the frame come here
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)



    grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_100)
    parameters =  aruco.DetectorParameters_create()
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    frame_markers = aruco.drawDetectedMarkers(frame.copy(), corners, ids)


    for rejectedPolygons in rejectedImgPoints:
         for points in rejectedPolygons:
            cv2.line(frame_markers, tuple(points[0]), tuple(points[1]), [100, 0, 100])
            cv2.line(frame_markers, tuple(points[2]), tuple(points[1]), [100, 0, 100])
            cv2.line(frame_markers, tuple(points[2]), tuple(points[3]), [100, 0, 100])
            cv2.line(frame_markers, tuple(points[0]), tuple(points[3]), [100, 0, 100])

#    cv2.imshow('frame_marker',frame_markers)
  #  "53" is the index id of that particular Aruco Marker ( tetsed with 4X4 matrix marker)
    if (ids==53):
       print("Now let's land")
        
       takeoff_land(vehicle, "land", 7)

       break

cap.release()
cv2.destroyAllWindows()