#!/usr/bin/env python


from __future__ import print_function
import time
from dronekit_sitl import SITL
from dronekit import Vehicle, connect, VehicleMode, LocationGlobalRelative
import math
import random
from datetime import datetime, date


'''
Returns the next coordinates for the drones while moving around the square
'''
def moveAroundSquare(currCoords, direc, delta):
    newCoords = LocationGlobalRelative(currCoords.lat, currCoords.lon, currCoords.alt)

    # MODIFY COORDINATES BASED ON WHICH CORNER
    # up
    if direc == 1:
        newCoords = LocationGlobalRelative(currCoords.lat + delta, currCoords.lon, currCoords.alt)

    # right
    elif direc == 2:
        newCoords = LocationGlobalRelative(currCoords.lat, currCoords.lon - delta, currCoords.alt)

    # down
    elif direc == 3:
        newCoords = LocationGlobalRelative(currCoords.lat - delta, currCoords.lon, currCoords.alt)

    # left
    elif direc == 4:
        newCoords = LocationGlobalRelative(currCoords.lat, currCoords.lon + delta, currCoords.alt)

    # RETURN NEXT COORDS
    return newCoords


'''
Returns the next coordinates for the drones moving to the corners at start
'''
def moveToCorners(currCoords, direc, delta, delta_lon):
    newCoords = LocationGlobalRelative(currCoords.lat, currCoords.lon, currCoords.alt)

    # MODIFY COORDINATES BASED ON WHICH CORNER
    # top left
    if direc == 1:
        newCoords = LocationGlobalRelative(currCoords.lat + delta, currCoords.lon + delta, currCoords.alt)

    # top right
    elif direc == 2:
        newCoords = LocationGlobalRelative(currCoords.lat + delta, currCoords.lon - delta, currCoords.alt)

    # bottom left
    elif direc == 3:
        newCoords = LocationGlobalRelative(currCoords.lat - delta, currCoords.lon + delta, currCoords.alt)

    # bottom right
    elif direc == 4:
        newCoords = LocationGlobalRelative(currCoords.lat - delta, currCoords.lon - delta, currCoords.alt)

    # RETURN NEXT COORDS
    return newCoords


'''
Returns the next coordinates for the drone in the center
'''
def moveUpDown(currCoords, direc, delta):
    newCoords = LocationGlobalRelative(currCoords.lat, currCoords.lon, currCoords.alt)

    # up
    if direc == 1:
        newCoords = LocationGlobalRelative(currCoords.lat, currCoords.lon, currCoords.alt + delta)

    # down
    elif direc == 2:
        newCoords = LocationGlobalRelative(currCoords.lat, currCoords.lon, currCoords.alt - delta)

    # RETURN NEXT COORDS
    return newCoords

''' 
TEST

test = LocationGlobalRelative(41.698391, -86.233914, 10)
moveAroundSquare(test, 1, .00002)
'''
