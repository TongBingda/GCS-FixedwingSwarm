from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil # Needed for command message definitions
import time

def do_change_speed(throttle):
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, #command
        0, # confirmation
        0, # param 1, Speed type (0=Airspeed, 1=Ground Speed, 2=Climb Speed, 3=Descent Speed)	
        -1, # param 2, Speed (-1 indicates no change, -2 indicates return to default vehicle speed)
        throttle, # param 3, Throttle (-1 indicates no change, -2 indicates return to default vehicle throttle value)	
        0, 0, 0, 0)  # param 4 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)

if __name__ == "__main__":
    vehicle = connect("tcp:127.0.0.1:5762", wait_ready=True)
    while True:
        do_change_speed(45)
        print(vehicle.groundspeed)
        time.sleep(1)