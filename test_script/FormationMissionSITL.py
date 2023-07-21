"""
SITL Mission, test for RC channel override and distance / heading control.
Version 20230519
Created by tongbingda
"""
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil # Needed for command message definitions
import logging, serial, serial.tools.list_ports, datetime, time, math, sys, os

def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.

    This method is an approximation, and will not be accurate over large distances and close to the 
    earth's poles. It comes from the ArduPilot test code: 
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

def get_bearing_degree(aLocation1, aLocation2):
    """
    In general, your current heading will vary as you follow a great circle path (orthodrome);
    the final heading will differ from the initial heading by varying degrees according to distance and latitude.

    This formula is for the initial bearing (sometimes referred to as forward asimuth) which if followed in
    a straight line along a great-circle arc will take you from the start point to the end point.
    """
    phi1 = aLocation1.lat * math.pi / 180
    phi2 = aLocation2.lat * math.pi / 180
    lambda1 = aLocation1.lon * math.pi / 180
    lambda2 = aLocation2.lon * math.pi / 180
    y = math.sin(lambda2-lambda1) * math.cos(phi2)
    x = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(lambda2-lambda1)
    theta = math.atan2(y,x)
    bearing = (theta*180/math.pi + 360) % 360
    # print("bearing = %s" % bearing)
    return bearing

def calc_follower_dest(start_location, distance, bearing, altitude):
    """
    Given a start point and a distance d along constant bearing theta, this will calculate the destination point.
    If you maintain a constant bearing along a rhumb line, you will gradually spiral in towards one of the poles.
    start_location = LocationGlobalRelative(latitude, longitude, altitude)
    distance in meters
    bearing in degree
    altitude in meters
    """
    R = 6371393
    phi1 = start_location.lat * math.pi / 180
    lambda1 = start_location.lon * math.pi /180
    delta = distance / R
    theta = bearing * math.pi / 180
    delta_phi = delta * math.cos(theta)
    phi2 = phi1 + delta_phi
    delta_psi = math.log(math.tan(phi2/2 + math.pi/4) / math.tan(phi1/2 + math.pi/4))
    if abs(delta_psi) > 10e-12:
        q = delta_phi / delta_psi
    else:
        q = math.cos(phi1)
    delta_lambda = delta * math.sin(theta) / q
    lambda2 = lambda1 + delta_lambda

    latitude = phi2 * 180 / math.pi
    longitude = lambda2 * 180 / math.pi
    # print("latitude = %s" % latitude)
    # print("longitude = %s" % longitude)
    destination = LocationGlobalRelative(latitude, longitude, altitude)
    return destination

if __name__ == "__main__":
    # connect to leader and follower vehicle
    vehicle_leader = connect("tcp:127.0.0.1:5762", wait_ready=True)
    vehicle_follower = connect("tcp:127.0.0.1:5772", wait_ready=True)
    distance_last = get_distance_metres(vehicle_leader.location.global_relative_frame, vehicle_follower.location.global_relative_frame)
    while vehicle_follower.mode.name == "GUIDED":
        print("Leader heading (deg): %s" % vehicle_leader.heading)
        print("Follower heading (deg): %s" % vehicle_follower.heading)
        print("Leader speed (m/s): %s" % vehicle_leader.groundspeed)
        print("Follower speed (m/s): %s" % vehicle_follower.groundspeed)
        distance = get_distance_metres(vehicle_leader.location.global_relative_frame, vehicle_follower.location.global_relative_frame)
        print("Distance between leader and follower (m): %s" % distance)
        bearing = get_bearing_degree(vehicle_follower.location.global_relative_frame, vehicle_leader.location.global_relative_frame)
        follower_dest = calc_follower_dest(vehicle_leader.location.global_relative_frame, 100, bearing, 90)
        vehicle_follower.simple_goto(follower_dest)
        vehicle_leader.channels.overrides = {'3':1600}

        desire_dist = 10
        kp = 10
        kd = 1
        channel_overrides = 1500 + kp*(distance-desire_dist) #+ kd*(distance_last-distance)
        if channel_overrides > 1900:
            channel_overrides = 1900
        elif channel_overrides < 1500:
            channel_overrides = 1500
        print("Channel overrides : %s" % channel_overrides)
        vehicle_follower.channels.overrides = {'3':int(channel_overrides)}


        distance_last = distance
        time.sleep(0.1)
        os.system("cls")