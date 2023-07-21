"""
FMS Ranger 1800mm Mission, Test For Automatic Takeoff, Landing And Waypoint Mission
Version 20230512
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

if __name__ == "__main__":
    # Simulation flag
    sitl_debug = False
    # System and logger config
    start_time = time.time()
    os.system("cls")
    os.system("color 0a")
    logger = logging.getLogger(__file__)
    logger.setLevel(logging.DEBUG)
    logfile_name = datetime.datetime.now().strftime("%Y%m%d-%H-%M-%S") + ".log"
    fh = logging.FileHandler(logfile_name)
    fh.setLevel(logging.INFO)
    ch = logging.StreamHandler()
    ch.setLevel(logging.DEBUG)
    formatter = logging.Formatter("%(asctime)s: %(message)s")
    ch.setFormatter(formatter)
    fh.setFormatter(formatter)
    logger.addHandler(ch)
    logger.addHandler(fh)

    if sitl_debug == True:
        sitl_device = "tcp:127.0.0.1:5762"
    else:
        # Check serial ports
        ports_list = list(serial.tools.list_ports.comports())
        if len(ports_list) <= 0:
            logger.info("No serial ports.")
            sys.exit()
        else:
            print("Available serial ports:")
            for comport in ports_list:
                print(list(comport)[0], list(comport)[1])
                if comport.manufacturer == "Silicon Laboratories" and comport.serial_number == 1001: # 915MHz radio telementry serial number = 1001
                    telem_device = comport.device
    
    # Connect vehicle
    if sitl_debug == True:
        vehicle = connect(sitl_device, wait_ready=True)
    else:
        max_retry = 10
        for retry_num in range(1,max_retry):
            try:
                vehicle = connect(telem_device, baud=57600, wait_ready=True)
            except:
                logger.info("Vehicle failed to connected, try %s time." % retry_num)
                if retry_num == max_retry - 1:
                    sys.exit()
            else:    
                print("Vehicle connected.")
                break

    # Mission config
    takeoff_alt = 40
    waypoints = [LocationGlobalRelative(39.3698986, 115.9155303, 50), 
                 LocationGlobalRelative(39.3690256, 115.9155303, 50), 
                 LocationGlobalRelative(39.3682086, 115.9155357, 50)]
    waypoint_next = 0
    waypoint_radius = 100
    max_loop = 5
    
    # Waiting for takeoff complete, in guided mode
    while True:
        if vehicle.location.global_relative_frame.alt > takeoff_alt and vehicle.mode.name == "GUIDED":
            loop_count = 0
            break
        print("Waiting for vehicle takeoff.")
        time.sleep(2)

    # Flight loop
    while True:
        # Log information
        run_time = time.time() - start_time
        os.system("cls")
        logger.info("--------New Control Loop--------")
        logger.info("Program run time: %s" % run_time)
        logger.info(vehicle.battery)
        logger.info(vehicle.mode)
        logger.info(vehicle.location.global_relative_frame)
        logger.info(vehicle.gps_0)
        logger.info("Next waypoint: %s" % waypoint_next)
        logger.info("Loop count: %s" % loop_count)

        # Flight mission
        if get_distance_metres(vehicle.location.global_relative_frame, waypoints[waypoint_next]) <= waypoint_radius and vehicle.mode.name == "GUIDED":
            if waypoint_next == len(waypoints) - 1:
                waypoint_next = 0
                loop_count = loop_count + 1
            else:
                waypoint_next = waypoint_next + 1
            vehicle.simple_goto(waypoints[waypoint_next])
        elif get_distance_metres(vehicle.location.global_relative_frame, waypoints[waypoint_next]) > waypoint_radius and vehicle.mode.name == "GUIDED":
            vehicle.simple_goto(waypoints[waypoint_next])

        # Exit judgement
        time.sleep(2)
        if vehicle.mode.name != "GUIDED":
            logger.info("Exit guided mode, mission aborted.")
            break
        if loop_count > max_loop and vehicle.battery.level <= 25:
            logger.info("Mode change to auto, vehicle return to land.")
            vehicle.mode = VehicleMode("AUTO")
    vehicle.close()