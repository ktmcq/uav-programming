#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
© Copyright 2015-2016, 3D Robotics.
simple_goto.py: GUIDED mode "simple goto" example (Copter Only)
Demonstrates how to arm and takeoff in Copter and how to navigate to points using Vehicle.simple_goto.
Full documentation is provided at http://python.dronekit.io/examples/simple_goto.html
"""

import time
import math
from dronekit_sitl import SITL
from dronekit import Vehicle, VehicleMode, connect, LocationGlobalRelative
from dance1 import moveAroundSquare, moveToCorners, moveUpDown

XDANCE = True  

copters = []
sitls = []
drone_models = []

lat_delta = .0003
lon_delta = .0003
alt_delta = 1.


# get the distance from two locations in meters
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


def connect_virtual_vehicle(instance, home):
    sitl = SITL()
    sitl.download('copter', '3.3', verbose=True)
    instance_arg = '-I%s' %(str(instance))
    print("Drone instance is: %s" % instance_arg)
    home_arg = '--home=%s, %s,%s,180' % (str(home[0]), str(home[1]), str(home[2]))
    sitl_args = [instance_arg, '--model', 'quad', home_arg]
    sitl.launch(sitl_args, await_ready=True)
    tcp, ip, port = sitl.connection_string().split(':')
    port = str(int(port) + instance * 10)
    conn_string = ':'.join([tcp, ip, port])
    print('Connecting to vehicle on: %s' % conn_string)

    vehicle = connect(conn_string)
    vehicle.wait_ready(timeout=120)

    # Collections
    copters.append(vehicle)
    sitls.append(sitl)

def copters_at_altitude(aTargetAltitude):
    while True:
        at_altitude = True
        ctr=1
        for c in copters:
            print ('Copter ID: {} at altitude {} '.format(ctr,str(c.location.global_relative_frame.alt))) 
            ctr = ctr + 1
            if (not c.location.global_relative_frame.alt >= aTargetAltitude * 0.95):
                at_altitude = False
        time.sleep(3)

        if at_altitude == True:
            print("All drones have reached their target altitudes")
            break     

def copters_arm():
    for c in copters:
        c.mode = VehicleMode("GUIDED")
        c.armed = True

    for c in copters:
        while not (c.armed):
            time.sleep(1)

def land_drones():
    for c in copters:
        c.mode = VehicleMode("LAND")
    print ("LANDING....")
    time.sleep(30)

def copters_armable():
 
    while True:
        unarmable = False
        for c in copters:
            if (not c.is_armable):
                unarmable = True
        time.sleep(3)

        if unarmable == False:
            break     


def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """
    print("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    copters_armable()
 
    print("Arming motors")
    copters_arm()
  
    print("Vehicle armed!")

    print("All drones are now Taking off!")
    aTargetAltitude = 10
    for c in copters:
       	c.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    print("Waiting for copters to ascend")
    copters_at_altitude(aTargetAltitude)

def are_copters_guided():
	result = True
	for c in copters:
		if c.mode.name == "GUIDED":
			result = False
	
	return result

# Starting starting_coordinates
base_coordinate = [41.698391, -86.2233914,0]
starting_coordinates = []
coordinates = []

# Copter list
# four corners dance
if XDANCE:
	# Top Left
	coord2 = [base_coordinate[0] + 0.0001, base_coordinate[1] + 0.0001, 0]
	
	# Top Right
	coord3 = [base_coordinate[0] + 0.0001, base_coordinate[1] - 0.0001, 0]
	
	# Bottom Left 
	coord4 = [base_coordinate[0] - 0.0001, base_coordinate[1] + 0.0001, 0]
	
	# Bottom Right
	coord5 = [base_coordinate[0] - 0.0001, base_coordinate[1] - 0.0001, 0]
	
	# Base Coordinate is the Middle
	starting_coordinates.append(base_coordinate)
	starting_coordinates.append(coord2)
	starting_coordinates.append(coord3)
	starting_coordinates.append(coord4)
	starting_coordinates.append(coord5)
	
	for n in range(5):
		connect_virtual_vehicle(n, starting_coordinates[n])

    # Main loop
	while are_copters_guided():
		# send the non-center drones to the corners
		for i in range(0, 5):
		# calculate next coordinates
			coords = []
			coords.append(moveUpDown(copters[0].location.global_relative_frame, 1, alt_delta))
			coords.append(moveToCorners(copters[1].location.global_relative_frame, 1, lat_delta, lon_delta))
			coords.append(moveToCorners(copters[2].location.global_relative_frame, 2, lat_delta, lon_delta))
			coords.append(moveToCorners(copters[3].location.global_relative_frame, 3, lat_delta, lon_delta))
			coords.append(moveToCorners(copters[4].location.global_relative_frame, 4, lat_delta, lon_delta))
        
			# send drones to next coordinates
			for i in range(0, 5):
				copters[i].simple_goto(coords[i], groundspeed=15)

			# wait until they all get to their waypoints
			there_yet = [0, 0, 0, 0, 0]
			while are_copters_guided():
				for i in range(0, 5):
					remaining_distance = get_distance_meters(copters[i].location.global_relative_frame, coords[i])
					print(remaining_distance)
					if remaining_distance <= .5:
						there_yet[i] = 1
					else:
						there_yet[i] = 0
				if sum(there_yet) == 5:
					break

			# now move around the square
			print("time to move")


# line dance
else:
	for n in range(5):
		coordinates = [coordinates[0], \
		coordinates[1]-(0.00005*n),coordinates[2]]
		connect_virtual_vehicle(n,starting_coordinates)


# Arm and takeoff to 10 meters
arm_and_takeoff(10) 



# Land them
land_drones()

# Close all vehicles
for c in copters:
  c.close()

# Shut down simulators
for s in sitls:
    s.stop()

