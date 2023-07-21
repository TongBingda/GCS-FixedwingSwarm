"""
FMS Ranger 1800mm Mission, Test For Automatic Takeoff, Landing And Waypoint Mission
2023.05.25 New Test For Matplotlib 3D Realtime Trajectory Show
Version 20230525
Created by tongbingda
"""
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil # Needed for command message definitions
from collections import namedtuple
import matplotlib.animation as animation
import matplotlib.pyplot as plt
import logging, threading, serial, serial.tools.list_ports, datetime, time, math, sys, os

def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.

    This method is an approximation, and will not be accurate over large distances and close to the 
    earth's poles. It comes from the ArduPilot test code: 
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py

    *** UPDATE 20230525 ***
    This uses the 'haversine' formula to calculate the great-circle distance between two points - 
    that is, the shortest distance over the earth's surface - giving an 'as-the-crow-flies' distance between the points.
    Details:
    https://movable-type.co.uk/scripts/latlong.html
    """
    R = 6371393
    phi1 = aLocation1.lat * math.pi / 180
    phi2 = aLocation2.lat * math.pi / 180
    delta_phi = (aLocation2.lat - aLocation1.lat) * math.pi /180
    delta_lambda = (aLocation2.lon - aLocation1.lon) * math.pi / 180
    a = math.sin(delta_phi / 2) * math.sin(delta_phi / 2) + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2) * math.sin(delta_lambda / 2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    d = R * c
    return d

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

def get_grid_location(aLocation1, aLocation2):
    """
    Convert longitude, latitude and altitude to x, y and z grid system (in meters)
    aLocation1 is the home location, aLocation2 is the target location
    """
    distance = get_distance_metres(aLocation1, aLocation2)
    bearing = get_bearing_degree(aLocation1, aLocation2) * math.pi / 180
    x = distance * math.sin(bearing)
    y = distance * math.cos(bearing)
    return x, y, aLocation2.alt

def update_animate(frame):
    x, y, z = get_grid_location(home_location, vehicle.location.global_relative_frame)
    vehicle_location.x.append(x)
    vehicle_location.y.append(y)
    vehicle_location.z.append(z)
    ax.cla()
    ax.set_aspect("equal")
    ax.set_xlabel("x position")
    ax.set_ylabel("y position")
    ax.set_zlabel("z position")
    ax.set_title("Vehicle 3D Trajectory")
    ax.plot3D(vehicle_location.x, vehicle_location.y, vehicle_location.z, "blue")
    
    if vehicle.mode.name != "GUIDED":
        return

def mission_thread(inteval):
    # Flight Mission loop
    while True:
        # global variables announcement
        global run_time, waypoint_next, loop_count, vehicle, stop_flag

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

        # Exit mission judgement
        if vehicle.mode.name != "GUIDED" or vehicle.location.global_relative_frame.alt < land_alt or stop_flag == True:
            logger.info("Exit guided mode, mission aborted.")
            break # Exit mission_thread() function
        if loop_count > max_loop:
            logger.info("Mode change to auto, vehicle return to land.")
            vehicle.mode = VehicleMode("AUTO")

        time.sleep(inteval)

if __name__ == "__main__":
    # Simulation flags
    sitl_debug = True
    
    # System and logger config
    start_time = time.time() # Start timer
    os.system("cls") # clear command window
    os.system("color 0a") # set command window color
    if not os.path.exists(os.path.dirname(__file__) + "/logs"):
        os.makedirs(os.path.dirname(__file__) + "/logs")
    logger = logging.getLogger(__file__)
    logger.setLevel(logging.DEBUG)
    logfile_name =os.path.dirname(__file__) + "/logs/" + os.path.basename(__file__).split(".")[0] + datetime.datetime.now().strftime("%Y%m%d-%H-%M-%S") + ".log"
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
        # SITL debugging, use tcp connection
        logger.info("SITL debugging, use TCP connection.")
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
        # Retry radio telemetry connection, need to modification later
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
    home_location = LocationGlobalRelative(39.3690256, 115.9155303, 0) # vehicle's home location = vehicle's current location
    # home_location = LocationGlobalRelative(39.3699914, 115.915406, 0) # vehicle's home location = fixed location
    takeoff_alt = 40
    land_alt = 5
    waypoints = [LocationGlobalRelative(39.3698986, 115.9155303, 50), 
                 LocationGlobalRelative(39.3690256, 115.9155303, 50), 
                 LocationGlobalRelative(39.3682086, 115.9155357, 50)]
    waypoint_next = 0
    waypoint_radius = 100
    max_loop = 1
    stop_flag = False
    flight_mission = threading.Thread(target=mission_thread, args=(1,)) # create flight mission thread

    # Matplolib configuration initialize
    fig = plt.figure()
    ax = plt.axes(projection="3d")
    Vehicle_Location = namedtuple("Vehicle_Location", ["x", "y", "z"])
    vehicle_x, vehicle_y, vehicle_z = get_grid_location(home_location, vehicle.location.global_relative_frame)
    vehicle_location = Vehicle_Location([vehicle_x], [vehicle_y], [vehicle_z])
    
    # Waiting for takeoff complete, in guided mode
    while True:
        if vehicle.location.global_relative_frame.alt > takeoff_alt and vehicle.mode.name == "GUIDED":
            loop_count = 0
            break
        print("Waiting for vehicle takeoff.")
        time.sleep(2)

    # start flight mission thread
    flight_mission.start()

    # Matplotlib realtime draw
    ani = animation.FuncAnimation(fig, update_animate)
    plt.tight_layout()
    plt.show()

    
    stop_flag = True
    run_time = time.time() - start_time
    logger.info("Stop mission thread at: %s" % run_time)

    # close vehicle object
    vehicle.close()