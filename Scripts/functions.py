#!/usr/bin/env python

import sys
import time
from math import sqrt, pow
from pymavlink import mavutil

def arm(vehicle):

    vehicle.mav.command_long_send(vehicle.target_system, vehicle.target_component,
                                     mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)

def takeoff_land(vehicle, choice, alt):

    if choice == "takeoff":
        vehicle.mav.command_long_send(vehicle.target_system, vehicle.target_component,
                                     mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, alt)
    else:
        vehicle.mav.command_long_send(vehicle.target_system, vehicle.target_component,
                                     mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, alt)

def change_mode(vehicle, mode):
    # Check if mode is available
    if mode not in vehicle.mode_mapping():
        print('Unknown mode : {}'.format(mode))
        print('Try:', list(vehicle.mode_mapping().keys()))
        sys.exit(1)

    # Get and Set mode ID
    mode_id = vehicle.mode_mapping()[mode]

    vehicle.set_mode(mode_id)

    time.sleep(4)

def goto_position_target_local_ned(vehicle, north, east, down):
    
    msg = mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
        0,       # time_boot_ms 
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111111000, # type_mask 
        north, east, down, # x, y, z positions 
        0, 0, 0, # x, y, z velocity in m/s  
        0, 0, 0, # x, y, z acceleration 
        0, 0)    

    vehicle.mav.send(msg)


    