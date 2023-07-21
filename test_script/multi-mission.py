from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil # Needed for command message definitions
import logging, serial, serial.tools.list_ports, datetime, time, math, os

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

if __name__ == "__main__":
    # Start timer
    start_time = time.time()

    # Check serial ports
    ports_list = list(serial.tools.list_ports.comports())
    if len(ports_list) <= 0:
        print("No serial ports.")
    else:
        print("Available serial ports:")
        for comport in ports_list:
            print(list(comport)[0], list(comport)[1])

    logfile_name = datetime.datetime.now().strftime("%Y%m%d-%H-%M-%S") + ".log"
    logging.basicConfig(filename=logfile_name, format='%(asctime)s: %(message)s', level=logging.DEBUG)
    os.system("color 0a")
    # Define 2 waypoints
    waypoints = [LocationGlobalRelative(39.3680137, 115.9136152, 100), LocationGlobalRelative(39.3679474, 115.9169197, 100), LocationGlobalRelative(39.3679308, 115.9186900, 100)]    
    waypoint_next = 0
    vehicle = connect("tcp:127.0.0.1:5762", wait_ready=True)
    logging.info("Length of waypoints: %s" % len(waypoints))
    while True:
        logging.info("Vehicle's mode: %s" % vehicle.mode.name)
        if vehicle.mode.name != "GUIDED":
            break
        flight_time = time.time() - start_time
        logging.info("Vehicle's flight time: %s" % flight_time)
        # if flight_time > 0.2*60:
            # vehicle.mode = VehicleMode("AUTO")
        logging.info("Vehicle's location: %s" % vehicle.location.global_relative_frame)
        logging.info("GPS: %s" % vehicle.gps_0)
        logging.info("Vehicle's next waypoint: %s" % waypoint_next)

        # Loop through waypoint missions
        if get_distance_metres(vehicle.location.global_relative_frame, waypoints[waypoint_next]) <= 100:
            if waypoint_next == len(waypoints) - 1:
                waypoint_next = 0
            else:
                waypoint_next = waypoint_next + 1
            vehicle.simple_goto(waypoints[waypoint_next], groundspeed=25)
        else:
            vehicle.simple_goto(waypoints[waypoint_next], groundspeed=25)

        time.sleep(1)
        os.system("cls")
    vehicle.close()