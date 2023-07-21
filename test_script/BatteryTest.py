from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil # Needed for command message definitions
import logging, serial, serial.tools.list_ports, datetime, time, math, sys, os

if __name__ == "__main__":
    vehicle = connect("COM3", baud=57600, wait_ready=True)
    print(vehicle.battery.level)
    print(vehicle.battery.voltage)