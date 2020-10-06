from dronekit import connect, VehicleMode, time
#from __future__ import print_function
import time
from dronekit_sitl import SITL
from dronekit import Vehicle, VehicleMode, connect, LocationGlobalRelative
import math
import json
from websocket import create_connection
from drone_model import Drone_Model
import argparse

parser = argparse.ArgumentParser(description='Print out vehicle state information')
parser.add_argument('--connect',help="vehicle connection target string.")
args=parser.parse_args()

connection_string = args.connect

# Connect to the Vehicle.
print("\nConnecting to vehicle on: %s" % connection_string)
vehicle = connect(connection_string, wait_ready=False)
vehicle.wait_ready(timeout=120)

#Get some vehicle attributes (state)
print("Get some vehicle attribute values:")
print("GPS: %s" % vehicle.gps_0)
print("Battery: %s" % vehicle.battery)
print("Last Heartbeat: %s" % vehicle.last_heartbeat)
print("Is Armable?: %s" % vehicle.system_status.state)
print("Mode: %s" % vehicle.mode.name) # settable



####################################
# YOUR CODE GOES HERE
####################################
# Check that you are using proper GOTO commands that are
# executed in a loop with the following logic:
#def my_goto_function(target):
#    print the target coordinates so we know what is happening
#    simple goto command
#    while loop must include while vehicle.mode="GUIDED"
#        check distance to target
#        if close to target
#            break
#
# You can call your function for each waypoint
# SANITY CHECK that you don't go farther than 20 meters from your starting waypoint.
# I'm somehow against anyone trying to fly to Australia during our flight tests!

'''
globals
'''
ws = create_connection("ws://localhost:8000")
drone_model_object =  Drone_Model(1,0,0)


'''
functions
'''
# calculate new coords based on current coords and lat and lon distances
def get_new_coords(curr_loc, d_lat, d_lon):
    earth_R = 6371
    new_lat = curr_loc.lat + d_lat * (1 / ((2*math.pi/360)*earth_R)) / 1000
    new_lon = curr_loc.lon + d_lon * (1 / ((2*math.pi/360)*earth_R)) / 1000 / math.cos(curr_loc.lat * math.pi / 180)

    return new_lat, new_lon


# get the distance in meters from loc1 to loc2
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


def custom_sleep(drone_model, sleep_time):
    current_time = 0
    while(current_time<sleep_time):
        lat = vehicle.location.global_relative_frame.lat
        lon = vehicle.location.global_relative_frame.lon
        drone_model.update_status(lat,lon)
        ws.send(drone_model.toJSON())
        print('Current location is: {0},{1}'.format(lat,lon))
        time.sleep(1)
        current_time+=1


def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """
    print("Basic pre-arm checks")
    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    print("Vehicle armed!")

    print("Taking off!")
    lat = vehicle.location.global_relative_frame.lat
    lon = vehicle.location.global_relative_frame.lon
    print('Current location before takeoff is: {0},{1}'.format(lat,lon))
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude
    lat = vehicle.location.global_relative_frame.lat
    lon = vehicle.location.global_relative_frame.lon
    print('Current location after takeoff is: {0},{1}'.format(lat,lon))

    # Wait until the vehicle reaches a safe height before processing the goto
    #  (otherwise the command after Vehicle.simple_takeoff will execute
    #   immediately).
    while vehicle.mode.name=="GUIDED":
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        drone_model_object.update_status(vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon)
        ws.send(drone_model_object.toJSON())
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)


'''
main execution
'''
# take off to 10 meters
arm_and_takeoff(10)


# hover for 5 seconds
time.sleep(5)


# gly north west for approximately 20 meters
new_lat, new_lon = get_new_coords(vehicle.location.global_relative_frame, 20, -20)
new_loc = LocationGlobalRelative(new_lat, new_lon, 10)
vehicle.simple_goto(new_loc)
while vehicle.mode.name=="GUIDED":
    print(" Location: ", vehicle.location.global_relative_frame)
    drone_model_object.update_status(vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon)
    ws.send(drone_model_object.toJSON())
    # Break and return from function just below target altitude.
    if get_distance_meters(vehicle.location.global_relative_frame, new_loc) <= 1.5:
        print("Reached waypoint")
        break
    time.sleep(1)


# increase altitude to 15 meters
new_loc2 = LocationGlobalRelative(new_loc.lat, new_loc.lon, 15)
vehicle.simple_goto(new_loc2)
while vehicle.mode.name=="GUIDED":
    print(" Altitude: ", vehicle.location.global_relative_frame.alt)
    drone_model_object.update_status(vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon)
    ws.send(drone_model_object.toJSON())
    # Break and return from function just below target altitude.
    if vehicle.location.global_relative_frame.alt >= 15 * 0.95:
        print("Reached target altitude")
        break
    time.sleep(1)


# fly east for approximately 20 meters
new_lat, new_lon = get_new_coords(vehicle.location.global_relative_frame, 0, 20)
new_loc3 = LocationGlobalRelative(new_lat, new_lon, 15)
vehicle.simple_goto(new_loc3)
while vehicle.mode.name=="GUIDED":
    print(" Location: ", vehicle.location.global_relative_frame)
    drone_model_object.update_status(vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon)
    ws.send(drone_model_object.toJSON())
    # Break and return from function just below target altitude.
    if get_distance_meters(vehicle.location.global_relative_frame, new_loc3) <= 1.5:
        print("Reached waypoint")
        break
    time.sleep(1)


# fly back to launch point at 10 meters
home_lat = 41.714469
home_lon = -86.241786
new_loc4 = LocationGlobalRelative(home_lat, home_lon, 10)
vehicle.simple_goto(new_loc4)
while vehicle.mode.name=="GUIDED":
    print(" Location: ", vehicle.location.global_relative_frame)
    drone_model_object.update_status(vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon)
    ws.send(drone_model_object.toJSON())
    # Break and return from function just below target altitude.
    if get_distance_meters(vehicle.location.global_relative_frame, new_loc4) <= 1.5:
        print("Reached waypoint")
        break
    time.sleep(1)



####################################
# Add this at the end
####################################
print("Landing")
vehicle.mode = VehicleMode("LAND")  # Note we replace RTL with LAND

# Close vehicle object before exiting script
print("Close vehicle object")
vehicle.close()

# Close vehicle object before exiting script
vehicle.close()

time.sleep(5)

print("Completed")
