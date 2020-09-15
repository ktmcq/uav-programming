#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
© Copyright 2015-2016, 3D Robotics.
simple_goto.py: GUIDED mode "simple goto" example (Copter Only)
Demonstrates how to arm and takeoff in Copter and how to navigate to points using Vehicle.simple_goto.
Full documentation is provided at http://python.dronekit.io/examples/simple_goto.html
"""

from __future__ import print_function
import time
from dronekit_sitl import SITL
from dronekit import Vehicle, connect, VehicleMode, LocationGlobalRelative
import math
import random
from datetime import datetime, date


# GLOBALS
# latitudes
top = 41.715167
bottom = 41.714350
# longitudes
right = -86.240335
left = -86.243146
# sleep
nap = .2
# waypoints array
waypoints = []


# FUNCTIONS (and some usages of the functions)
# hide black box
def hide_black_box():
    vRange = top - bottom
    hRange = right - left

    # create latitude
    randNum = random.random()
    vOffset = randNum * vRange
    newLat = bottom + vOffset
    newLat = '%.6f'%(newLat)

    # create longitude
    randNum = random.random()
    hOffset = randNum * hRange
    newLon = left + hOffset
    newLon = '%.6f'%(newLon)

    print('Black Box Hidden')
    return (newLat, newLon, 0)

# hide black box and write to file
hiddenCoords = hide_black_box()
hiddenSpot = LocationGlobalRelative(float(hiddenCoords[0]), float(hiddenCoords[1]), float(hiddenCoords[2]))
print("Black Box: " + str(hiddenSpot))
out_file = open("uav_output.txt", "w")
out_file.write("Black box location: " + str(hiddenSpot.lat) + ", " + str(hiddenSpot.lon))


# get the distance from 
def get_distance_meters(loc1, loc2):
    # get lats and lons
    lat_drone = loc1.lat
    lon_drone = loc1.lon
    lat_box = loc2.lat
    lon_box = loc2.lon

    # calculate and return distance
    R = 6371
    dLat = lat_box * math.pi/180 - lat_drone * math.pi/180
    dLon = lon_box * math.pi/180 - lon_drone * math.pi/180
    a = math.sin(dLat/2) * math.sin(dLat/2) + math.cos(lat_drone * math.pi/180) * math.cos(lat_box * math.pi/180) * math.sin(dLon/2) * math.sin(dLon/2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    d = R * c * 1000;
    return d

topleft = LocationGlobalRelative(top, left, 0)
botleft = LocationGlobalRelative(bottom, left, 0)
botright = LocationGlobalRelative(bottom, right, 0)
height = get_distance_meters(topleft, botleft)
width = get_distance_meters(botleft, botright)
print("Field dimensions: " + str(width) + " x " + str(height) + " meters")


# ping; returns location if within 5m of black box
def ping(loc_drone, loc_box):
    # if within 5 meters of black box
    if get_distance_meters(loc_drone, loc_box) <= 5:
        return loc_box

    # not within 5 meters of black box
    else:
        return


# arm and take off
def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(3)
        print("Arming motors")
        vehicle.mode = VehicleMode("GUIDED")
        vehicle.armed = True

    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)
    
    print("Vehicle armed!")
    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto
    #  (otherwise the command after Vehicle.simple_takeoff will execute
    #   immediately).
    while vehicle.mode.name=="GUIDED":
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            waypoints.append((vehicle.location.global_frame.lat, vehicle.location.global_frame.lon))
            break 


# find the black box
def find_black_box():
    # go to starting position for search (and search on way)
    print("Going towards top left corner...")
    target = LocationGlobalRelative(top, left, 10)
    vehicle.simple_goto(target, groundspeed=15)
    while vehicle.mode.name=="GUIDED":
        # check for black box on way to target
        loc = ping(vehicle.location.global_relative_frame, hiddenSpot)
        if loc:
            print("Black box found!")
            return loc

        # check distance to target
        remaining_distance = get_distance_meters(vehicle.location.global_frame, target)
        #print(remaining_distance)
        if remaining_distance <= .5:
            print("Waypoint reached")
            waypoints.append((vehicle.location.global_frame.lat, vehicle.location.global_frame.lon))
            break
        
        time.sleep(nap)

    # start the search algorithm
    print("Starting grid search...")
    lat_dist = top - bottom
    lon_dist = right - left
    way = "right"
    j = 1
    while vehicle.mode.name=="GUIDED" and vehicle.location.global_relative_frame.lat >= bottom:
        if way == "right":
            # go right
            i = 1
            while vehicle.mode.name=="GUIDED" and vehicle.location.global_relative_frame.lon <= right:
                new_lon = left + i*lon_dist/10.
                target = LocationGlobalRelative(vehicle.location.global_relative_frame.lat, new_lon, 10)
                vehicle.simple_goto(target, groundspeed=15)
                while vehicle.mode.name=="GUIDED":
                    # check for black box on way to target
                    loc = ping(vehicle.location.global_relative_frame, hiddenSpot)
                    if loc:
                        print("Black box found!")
                        return loc

                    # check distance to target
                    remaining_distance = get_distance_meters(vehicle.location.global_frame, target)
                    if remaining_distance <= .5:
                        print("Waypoint reached")
                        waypoints.append((vehicle.location.global_frame.lat, vehicle.location.global_frame.lon))
                        break

                    time.sleep(nap)
                i += 1

            # change direction
            way = "left"

        else:
            # go left
            i = 1
            while vehicle.mode.name=="GUIDED" and vehicle.location.global_relative_frame.lon >= left:
                new_lon = right - i*lon_dist/10.
                target = LocationGlobalRelative(vehicle.location.global_relative_frame.lat, new_lon, 10)
                vehicle.simple_goto(target, groundspeed=15)
                while vehicle.mode.name=="GUIDED":
                    # check for black box on way to target
                    loc = ping(vehicle.location.global_relative_frame, hiddenSpot)
                    if loc:
                        print("Black box found!")
                        return loc

                    # check distance to target
                    remaining_distance = get_distance_meters(vehicle.location.global_frame, target)
                    if remaining_distance <= .5:
                        print("Waypoint reached")
                        waypoints.append((vehicle.location.global_frame.lat, vehicle.location.global_frame.lon))
                        break

                    time.sleep(nap)
                i += 1

            # change direction
            way = "right"

        # move down
        print("Going down...")
        new_lat = top - j*lat_dist/13
        target = LocationGlobalRelative(new_lat, vehicle.location.global_relative_frame.lon, 10)
        vehicle.simple_goto(target, groundspeed=15)
        while vehicle.mode.name=="GUIDED":
            # check for black box on way to target
            loc = ping(vehicle.location.global_relative_frame, hiddenSpot)
            if loc:
                print("Black box found!")
                return loc

            # check distance to target
            remaining_distance = get_distance_meters(vehicle.location.global_frame, target)
            if remaining_distance <= .5:
                print("Waypoint reached")
                waypoints.append((vehicle.location.global_frame.lat, vehicle.location.global_frame.lon))
                break

            time.sleep(nap)
        j += 1



# MAIN EXECUTION
# Set up option parsing to get connection string
import argparse
parser = argparse.ArgumentParser(description='Commands vehicle using vehicle.simple_goto.')
parser.add_argument('--connect',
                    help="Vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()
connection_string = args.connect
sitl = None


# Start SITL if no connection string specified
# This technique for starting SITL allows us to specify defffaults 
if not connection_string:
    sitl_defaults = '~/git/ardupilot/tools/autotest/default_params/copter.parm'
    sitl = SITL()
    sitl.download('copter', '3.3', verbose=True)
    sitl_args = ['-I0', '--model', 'quad', '--home=41.714841,-86.241941,0,180']
    sitl.launch(sitl_args, await_ready=True, restart=True)
    connection_string = 'tcp:127.0.0.1:5760'

# Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True, baud=57600)
print ('Current position of vehicle is: %s' % vehicle.location.global_frame)


# Arm and take off
start = datetime.now().replace(microsecond=0)
arm_and_takeoff(10) # go to relative altitude of 10m

# Once at high enough altitude, find the box
target_coords = find_black_box()

# if black box found, travel to it
if target_coords:
    target_coords.alt = 10
    vehicle.simple_goto(target_coords, groundspeed=15)
    # check distance to target
    while vehicle.mode.name=="GUIDED":
        remaining_distance = get_distance_meters(vehicle.location.global_frame, target_coords)
        if remaining_distance <= .5:
            print("Above black box!")
            waypoints.append((vehicle.location.global_frame.lat, vehicle.location.global_frame.lon))
            break
        time.sleep(nap)
    print("Landing...")
    vehicle.mode = VehicleMode("LAND")

else:
    print("Black box not found; Return to Launch")
    vehicle.mode = VehicleMode("RTL")


# lower down onto black box
final_lat = vehicle.location.global_frame.lat
final_lon = vehicle.location.global_frame.lon
waypoints.append((final_lat, final_lon))
out_file.write("\nUAV landing location: " + str(final_lat) + ", " + str(final_lon))


# print time data
end = datetime.now().replace(microsecond=0)
out_file.write("\n\nStarting time: " + str(start))
out_file.write("\nEnding time: " + str(end))
out_file.write("\nTime taken to locate box: " + str(end - start))


# print log of waypoints
out_file.write("\n\nLog of waypoints: ")
out_file.write(waypoints)


# Close vehicle object before exiting script
print("Close vehicle object")
vehicle.close()

# Shut down simulator if it was started.
if sitl:
    sitl.stop()
