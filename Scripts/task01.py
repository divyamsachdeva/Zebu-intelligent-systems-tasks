#!/usr/bin/python

from functions import *
from pymavlink import mavutil
import sys

# Connection to the vehicle
vehicle = mavutil.mavlink_connection('udpin:localhost:14551')
vehicle.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" %
      (vehicle.target_system, vehicle.target_component))

mode = 'GUIDED'

change_mode(vehicle, mode)

""" Mission starts here """

arm(vehicle)

takeoff_land(vehicle, "takeoff", 5)

time.sleep(10)

goto_position_target_local_ned(vehicle, 10, 10, -5)

time.sleep(4)

takeoff_land(vehicle, "land", 5)

