from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil # Needed for command message definitions
from collections import namedtuple
import matplotlib.animation as animation
import matplotlib.pyplot as plt
import logging, serial, serial.tools.list_ports, datetime, time, math, sys, os

def anyargs(*args, **kwargs):
    print(args)
    print(kwargs)

if __name__ == "__main__":
    anyargs([1,2,3],xa=2)
    
    # vehicle = connect("tcp:127.0.0.1:5762", wait_ready=True)

    # while True:
    #     print(vehicle.servos.servo_raw)
    #     time.sleep(1)