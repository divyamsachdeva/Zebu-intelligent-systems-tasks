#!/usr/bin/python

from functions import *
from pymavlink import mavutil
import math
import numpy as np

# Connection to the vehicle
vehicle = mavutil.mavlink_connection('udpin:localhost:14551')
vehicle.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" %
      (vehicle.target_system, vehicle.target_component))

mode = 'GUIDED'

change_mode(vehicle, mode)

""" Mission starts here """

arm(vehicle)

takeoff_land(vehicle, "takeoff", 0.3)

time.sleep(10)

for t in np.arange(0.3,2*math.pi,0.3):
      # print(t)
      x = math.cos(6*t)
      y = math.sin(6*t)
      z = t
      # lst = [x,y,z]
      # print(lst)
      goto_position_target_local_ned(vehicle, x, y, -z)

      time.sleep(3)

takeoff_land(vehicle, "land", z)