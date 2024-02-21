"""
FMS Ranger 1800mm Mission, Test For Automatic Takeoff, Landing And Waypoint Mission

2023.06.28 Start building UAV ground control stations with pysimplegui 
and abandon the solution of directly using matplotlib to plot UAV trajectory

2023.07.21 Code that accomplishes two different task functions

2024.01.05 In the mission 2, the UAV swarm pursuit process is realized, 
and the SITL simulation verification is carried out.

Version 20240105
Created by tongbingda
"""
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative, Command
from pymavlink import mavutil # Needed for command message definitions
from collections import namedtuple
# import matplotlib.animation as animation
# import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, FigureCanvasAgg
from matplotlib.figure import Figure
import PySimpleGUI as sg
import logging, threading, serial, serial.tools.list_ports, pyttsx3, datetime, time, math, sys, os, random, socket

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

def socket_send(ip, msg):
    """
    The purpose of this Python code is to send data using UDP sockets. 
    UDP is a connectionless transport protocol that is used to transmit data 
    packets over the Internet. Unlike TCP, UDP does not guarantee data integrity.

    Implementation principle:
    Import the socket module. Create a UDP socket. Bind the local address and port.
    Set the remote address and port. Send data. Close the socket.Returns the number of bytes sent.
    
    Purpose:
    The primary purpose of this code is to send data to a remote computer. 
    It can be used for simple communication, file transfer, video streaming and other applications.
    
    Note:
    Make sure the port number of the remote computer is correct and open.
    Ensure that the IP address and port number of the local computer are correct and open.
    Make sure that the data you are sending is encoded correctly (for example, if you are sending a string, encode it as UTF-8).
    Since UDP does not guarantee data integrity, both the sender and receiver need to verify that the data is correct.
    """
    # Create a UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # Set the sender's address
    local_address = (ip, 20000)  # Local address and port
    sock.bind(local_address)
    # Set the receiver's address
    remote_address = (ip, 10000)  # Remote address and port
    # Send data
    sent_count = sock.sendto(msg.encode(), remote_address)
    # Close the socket
    sock.close()
    return sent_count

def draw_figure(canvas, figure, loc=(0, 0)):
    figure_canvas_agg = FigureCanvasTkAgg(figure, canvas)
    figure_canvas_agg.draw()
    figure_canvas_agg.get_tk_widget().pack(side='top', fill='both', expand=1)
    return figure_canvas_agg

def main_control_panel():
    # define the form layout
    # vehicle global control layout
    global_control = [
        # column 1
        [
            sg.Button("Arm", key="-GlobalControl ARM-"),
            sg.Button("Disarm", key="-GlobalControl DISARM-"),
            sg.Button("Land Sequence", key="-GlobalControl LANDSEQ-"),
            sg.Button("Emergency Stop", key="-GlobalControl EMERGENCYSTOP-")
        ],
        # column 2
        [
            sg.Button("TAKEOFF Mode", key="-GlobalControl TAKEOFF-"),
            sg.Button("FBWA Mode", key="-GlobalControl FBWA-"),
            sg.Button("GUIDED Mode", key="-GlobalControl GUIDED-"), 
            sg.Button("RTL Mode", key="-GlobalControl RTL-")
        ],
        # column 3
        [
            sg.Button("MANUAL Mode", key="-GlobalControl MANUAL-"), 
            sg.Button("AUTO Mode", key="-GlobalControl AUTO-"), 
            sg.Button("QLOITER Mode", key="-GlobalControl QLOITER-"), 
            sg.Button("QLAND Mode", key="-GlobalControl QLAND-"), 
        ],
        # column 4
        [
            sg.Text("Rally Point:"), sg.Combo(["home_location", "rally_point_1", "rally_point_2", "rally_point_3"], default_value="home_location"), sg.Button("Execute")
        ],
        # column 5
        [
            sg.Button("Start Mission 1", key="-Start Mission 1-", disabled=True), 
            sg.Button("Abort Mission 1", key="-Abort Mission 1-", disabled=True),
            sg.Button("Start Mission 2", key="-Start Mission 2-", disabled=False),
            sg.Button("Abort Mission 2", key="-Abort Mission 2-", disabled=False)
        ],
        # column 6
        [
            sg.Radio("Stage 1", "Mission Stage", default=True, key="-Mission Stage 1-"),
            sg.Radio("Stage 2", "Mission Stage", default=False, key="-Mission Stage 2-"),
            sg.Radio("Stage 3", "Mission Stage", default=False, key="-Mission Stage 3-"),
            sg.Radio("Stage 4", "Mission Stage", default=False, key="-Mission Stage 4-"),
            sg.Radio("Stage 5", "Mission Stage", default=False, key="-Mission Stage 5-"),
            sg.Checkbox("CatchUp", default=False, disabled=False, text_color="black",background_color="white", key="-CatchUp Flag-")
        ]
    ]
    # left layout: control panel
    layout_l = [
        [sg.Button("Refresh Ports", key="-Refresh Ports-"), sg.Text("Ports:", justification="left"), sg.Combo([], size=(18,1), key="-Ports-"), sg.Button("Connect", key="-Connect-")],
        [sg.Text("Vehicle Color:"), sg.Radio("Red", "Team Radio", default=True, key="-Radio Red-"), sg.Radio("Blue", "Team Radio", default=False, key="-Radio Blue-"), sg.Radio("Green", "Team Radio", default=False, key="-Radio Green-"), sg.Radio("Yellow", "Team Radio", default=False, key="-Radio Yellow-")],
        [sg.TabGroup([], key="-Tab Group-")],
        [sg.Frame("Global Control", global_control)]
    ]
    # right layout: display panel
    layout_r = [
        [sg.Canvas(size=(800, 800), key="-CANVAS-")]
    ]
    # main layout
    layout = [
        [sg.Text("固定翼无人机集群对抗地面站          ", font="黑体 24"), sg.Text("Port:"), sg.Input(default_text="tcp:127.0.0.1:5762", size=(15,1), key="-TCP Port-", enable_events=True), sg.Checkbox("Simulation/Base Station", enable_events=True, key="-SITL Debug-"), sg.Exit()],
        [sg.Column(layout_l, vertical_alignment="top"), sg.Column(layout_r)],
        # [sg.Column(layout_l, justification="left", element_justification="left", vertical_alignment="top"), sg.Column(layout_r, justification="right")],
        [sg.Button('Version'), sg.Button("Test"), sg.StatusBar(text="Information output here.", size=(50,1), key="-Status-")]
    ]
    
    window = sg.Window("UAV Swarm Control Panel", layout, finalize=True, keep_on_top=False, enable_close_attempted_event=True)

    return window

def control_tab(thisport):
    mode_list = ["MANUAL", "CIRCLE", "STABILIZE", "TRAINING", "ACRO", "FBWA", "FBWB",
                 "CRUISE", "AUTOTUNE", "AUTO", "RTL", "LOITER", "TAKEOFF", "GUIDED",
                 "QSTABILIZE", "QLOITER", "QLAND", "QRTL"]
    # vehicle control tab
    tab = [
            # new column 1
            [
                sg.Text("Device:"), sg.StatusBar(text="", size=(6,1), key="-"+thisport+" Device-"),
                sg.Text("SysID:"), sg.StatusBar(text="", size=(2,1), key="-"+thisport+" Sysid-"),
                sg.Text("Team:"), sg.StatusBar("", size=(6,1), key="-"+thisport+" Team-"),
                sg.Text("Battery:"), sg.StatusBar("",size=(4,1), key="-"+thisport+" Battery-"), sg.Text("V")
            ],
            # new column 2
            [
                sg.Text("System Status:"), sg.StatusBar("", size=(8,1), key="-"+thisport+" SysStatus-"),
                sg.Text("Armed:"), sg.StatusBar("", size=(5,1), key="-"+thisport+" Armed-"),
                sg.Button("Arm/Disarm", key="-"+thisport+" ArmDisarm-"),
                sg.Checkbox("Force", default=False, key="-"+thisport+" Force-")
            ],
            # new column 3
            [
                sg.Text("GPS FixType:"), sg.StatusBar("", size=(10,1), key="-"+thisport+" GPSFixType-"),
                sg.Text("Sat Num:"), sg.StatusBar("", size=(3,1), key="-"+thisport+" SatNum-"),
                sg.Checkbox("Disable GPS", default=False, enable_events=True, key="-"+thisport+" DisableGPS-")
            ],
            # new column 4
            [
                sg.Text("Mode:"), sg.StatusBar("", size=(6,1), key="-"+thisport+" Mode-"), 
                sg.Combo(mode_list, default_value="GUIDED", size=(10,1), key="-"+thisport+" ModeCombo-"), 
                sg.Button("Set Mode", key="-"+thisport+" SetMode-"),
                sg.Checkbox("Voice Warning", default=False, key="-"+thisport+" VoiceWarning-")
            ],
            # new column 5
            [
                sg.Text("Lat:"), sg.StatusBar("", size=(8,1), key="-"+thisport+" Lat-"), 
                sg.Text("Lon:"), sg.StatusBar("", size=(8,1), key="-"+thisport+" Lon-"), 
                sg.Checkbox("Live Plot", default=True, enable_events=True, key="-"+thisport+" LivePlot-")
            ],
            # new column 6
            [
                sg.Text("Alt:"), sg.StatusBar("", size=(8,1), key="-"+thisport+" Alt-"),
                sg.Text("Heading:"), sg.StatusBar("", size=(5,1), key="-"+thisport+" Heading-"),
                sg.Text("Groundspeed:"), sg.StatusBar("", size=(4,1), key="-"+thisport+" Groundspeed-"), sg.Text("m/s")
            ],
            # new column 7
            [
                sg.Text("Airspeed:"), sg.StatusBar("", size=(4,1), key="-"+thisport+" Airspeed-"), sg.Text("m/s"),
                sg.Button("Set Airspeed", key="-"+thisport+" AirspeedButton-"), sg.Input("12", size=(2,1), key="-"+thisport+" AirspeedInput-"), sg.Text("m/s")
            ],
            # new column 8
            [
                sg.Checkbox("Disable RC Failsafe", default=False, enable_events=True, key="-"+thisport+" DisableRCFailsafe-"),
                sg.Checkbox("Disable Serial 5 Protocol", default=False, enable_events=True, key="-"+thisport+" DisableSerial5Protocol-")
            ],
            # new column 9
            [
                sg.Text("CH1:"), sg.ProgressBar(800, size=(6,20), key="-"+thisport+" CH1-"),
                sg.Text("CH2:"), sg.ProgressBar(800, size=(6,20), key="-"+thisport+" CH2-"),
                sg.Text("CH3:"), sg.ProgressBar(800, size=(6,20), key="-"+thisport+" CH3-"),
                sg.Text("CH4:"), sg.ProgressBar(800, size=(6,20), key="-"+thisport+" CH4-")
            ],
            # new column 10
            [
                sg.Text("CH5:"), sg.ProgressBar(800, size=(6,20), key="-"+thisport+" CH5-"),
                sg.Text("CH6:"), sg.ProgressBar(800, size=(6,20), key="-"+thisport+" CH6-"),
                sg.Text("CH7:"), sg.ProgressBar(800, size=(6,20), key="-"+thisport+" CH7-"),
                sg.Text("CH8:"), sg.ProgressBar(800, size=(6,20), key="-"+thisport+" CH8-")
            ],
            # new column 11
            [
                sg.Text("Out1:"), sg.ProgressBar(800, size=(6,20), key="-"+thisport+" OUT1-"),
                sg.Text("Out2:"), sg.ProgressBar(800, size=(6,20), key="-"+thisport+" OUT2-"),
                sg.Text("Out3:"), sg.ProgressBar(800, size=(6,20), key="-"+thisport+" OUT3-"),
                sg.Text("Out4:"), sg.ProgressBar(800, size=(6,20), key="-"+thisport+" OUT4-")
            ],
            # new column 12
            [
                sg.Text("Out5:"),sg.ProgressBar(800, size=(6,20), key="-"+thisport+" OUT5-"),
                sg.Text("Out6:"),sg.ProgressBar(800, size=(6,20), key="-"+thisport+" OUT6-"),
                sg.Text("Out7:"),sg.ProgressBar(800, size=(6,20), key="-"+thisport+" OUT7-"),
                sg.Text("Out8:"),sg.ProgressBar(800, size=(6,20), key="-"+thisport+" OUT8-"),
            ],
            # new column 13
            [
                sg.Button("Land", key="-"+thisport+" Separate Land-"),
                sg.Button("Reboot Autopilot", key="-"+thisport+" Reboot-"),
                sg.Button("Disconnect "+thisport, key="-"+thisport+" Disconnect-")
            ]
            ]
    return tab

def run_pyttsx3(str):
    while engine._inLoop:
        time.sleep(0.2)
    engine.say(str)
    engine.runAndWait()

def update_state_tab(thisport, inteval=1, max_num=10):
    speed_warning_thread = threading.Thread(target=run_pyttsx3, args=(thisport+" 速度警告。",), daemon=True)
    altitude_warning_thread = threading.Thread(target=run_pyttsx3, args=(thisport+" 高度警告。",), daemon=True)
    while thisport in vehicles_port:
        # layout_l column 1
        window["-"+thisport+" Device-"].update(thisport) # The device name update code is temporarily placed here
        window["-"+thisport+" Sysid-"].update(vehicles[thisport].parameters["SYSID_THISMAV"])
        window["-"+thisport+" Team-"].update(vehicles_team[thisport])
        window["-"+thisport+" Battery-"].update(vehicles[thisport].battery.voltage)
        # layout_l column 2
        window["-"+thisport+" SysStatus-"].update(vehicles[thisport].system_status.state)
        if vehicles[thisport].armed == True:
            window["-"+thisport+" Armed-"].update("True")
        else:
            window["-"+thisport+" Armed-"].update("False")
        # layout_l column 3
        if vehicles[thisport].gps_0.fix_type == 0 == 0:
            window["-"+thisport+" GPSFixType-"].update("0 (No GPS)")
        elif vehicles[thisport].gps_0.fix_type == 1:
            window["-"+thisport+" GPSFixType-"].update("1 (No Fix)")
        elif vehicles[thisport].gps_0.fix_type == 2:
            window["-"+thisport+" GPSFixType-"].update("2 (2D Fix)")
        elif vehicles[thisport].gps_0.fix_type == 3:
            window["-"+thisport+" GPSFixType-"].update("3 (3D Fix)")
        elif vehicles[thisport].gps_0.fix_type == 4:
            window["-"+thisport+" GPSFixType-"].update("4 (3D GPS)")
        elif vehicles[thisport].gps_0.fix_type == 5:
            window["-"+thisport+" GPSFixType-"].update("5 (RTK Float)")
        elif vehicles[thisport].gps_0.fix_type == 6:
            window["-"+thisport+" GPSFixType-"].update("6 (RTK Fix)")
        window["-"+thisport+" SatNum-"].update(vehicles[thisport].gps_0.satellites_visible)
        # layout_l column 4
        window["-"+thisport+" Mode-"].update(vehicles[thisport].mode.name)
        if window["-"+thisport+" VoiceWarning-"].get() == True and vehicles[thisport].groundspeed < 10:
            if speed_warning_thread.is_alive() == False:
                speed_warning_thread = threading.Thread(target=run_pyttsx3, args=(thisport+" 速度警告。",), daemon=True)
                speed_warning_thread.start()
        if window["-"+thisport+" VoiceWarning-"].get() == True and vehicles[thisport].location.global_relative_frame.alt < 15:
            if altitude_warning_thread.is_alive() == False:
                altitude_warning_thread = threading.Thread(target=run_pyttsx3, args=(thisport+" 高度警告。",), daemon=True)
                altitude_warning_thread.start()
        # layout_l column 5
        window["-"+thisport+" Lat-"].update(vehicles[thisport].location.global_relative_frame.lat)
        window["-"+thisport+" Lon-"].update(vehicles[thisport].location.global_relative_frame.lon)
        # layout_l column 6
        window["-"+thisport+" Alt-"].update(vehicles[thisport].location.global_relative_frame.alt)
        window["-"+thisport+" Heading-"].update(vehicles[thisport].heading)
        window["-"+thisport+" Groundspeed-"].update(vehicles[thisport].groundspeed)
        # layout_l column 7
        window["-"+thisport+" Airspeed-"].update(vehicles[thisport].airspeed)
        # layout_l column 8
        # layout_l column 9
        
        window["-"+thisport+" CH1-"].update(current_count=vehicles[thisport].channels["1"])
        window["-"+thisport+" CH2-"].update(current_count=vehicles[thisport].channels["2"])
        window["-"+thisport+" CH3-"].update(current_count=vehicles[thisport].channels["3"])
        window["-"+thisport+" CH4-"].update(current_count=vehicles[thisport].channels["4"])
        # layout_l column 10
        window["-"+thisport+" CH5-"].update(current_count=vehicles[thisport].channels["5"])
        window["-"+thisport+" CH6-"].update(current_count=vehicles[thisport].channels["6"])
        window["-"+thisport+" CH7-"].update(current_count=vehicles[thisport].channels["7"])
        window["-"+thisport+" CH8-"].update(current_count=vehicles[thisport].channels["8"])
        # In order to be compatible with older versions of DroneKit, the Servo output is temporarily stopped from being updated here.
        # layout_l column 11
        # window["-"+thisport+" OUT1-"].update(current_count=vehicles[thisport].servos.servo_raw["1"])
        # window["-"+thisport+" OUT2-"].update(current_count=vehicles[thisport].servos.servo_raw["2"])
        # window["-"+thisport+" OUT3-"].update(current_count=vehicles[thisport].servos.servo_raw["3"])
        # window["-"+thisport+" OUT4-"].update(current_count=vehicles[thisport].servos.servo_raw["4"])
        # layout_l column 12
        # window["-"+thisport+" OUT5-"].update(current_count=vehicles[thisport].servos.servo_raw["5"])
        # window["-"+thisport+" OUT6-"].update(current_count=vehicles[thisport].servos.servo_raw["6"])
        # window["-"+thisport+" OUT7-"].update(current_count=vehicles[thisport].servos.servo_raw["7"])
        # window["-"+thisport+" OUT8-"].update(current_count=vehicles[thisport].servos.servo_raw["8"])
        # update vehicle location dictionary
        this_x, this_y, this_z = get_grid_location(origin_point, vehicles[thisport].location.global_relative_frame)
        location_x[thisport].append(this_x)
        location_y[thisport].append(this_y)
        if len(location_x[thisport]) > max_num:
            location_x[thisport].pop(0)
            location_y[thisport].pop(0)    

        # log vehicle's information
        # logger.info(thisport+": "+vehicles[thisport].location.global_relative_frame)
        time.sleep(inteval)

def update_animate(inteval):
    while True:
        ax.cla()
        ax.grid()
        ax.set_xlabel("X axis(m)")
        ax.set_ylabel("Y axis(m)")
        ax.grid("equal")
        for thisport in vehicles_port:
            if window["-"+thisport+" LivePlot-"].get() == True:
                ax.plot(location_x[thisport], location_y[thisport], color=vehicles_team[thisport], linewidth=2)
                # todo: ax.scatter will block old plot
                # ax.scatter(location_x[thisport][-1], location_y[thisport][-1], color=vehicles_team[thisport], s=80,marker="$"+str(vehicles_port.index(thisport)+1)+"$")

                # 20240216 for Unity 3D Simulation
                if len(location_x[thisport]) == 0:
                    continue
                else:
                    msg_id = vehicles_port.index(thisport) + 1
                    msg_type = 6
                    msg_model = 1
                    msg_status = 1
                    msg_camp = 1 if vehicles_team[thisport] == "red" else 2
                    msg_hierarchy = 1
                    msg_east = location_x[thisport][-1] * 100
                    msg_height = vehicles[thisport].location.global_relative_frame.alt * 100 + 1000
                    msg_north = location_y[thisport][-1] * 100
                    msg_pitch = vehicles[thisport].attitude.pitch * 180 / math.pi
                    msg_yaw = -vehicles[thisport].attitude.yaw * 180 / math.pi
                    msg_roll = vehicles[thisport].attitude.roll * 180 / math.pi
                    msg_speed = vehicles[thisport].airspeed
                    msg = "{id} {type} {model} {status} {camp} {hierarchy} {east} {height} {north} {pitch} {yaw} {roll} {speed};"\
                        .format(id=msg_id, type=msg_type, model=msg_model, status=msg_status, camp=msg_camp, hierarchy=msg_hierarchy,\
                                east=msg_east, height=msg_height, north=msg_north, pitch=msg_pitch, yaw=msg_yaw, roll=msg_roll, speed=msg_speed)
                    socket_send(local_ip, msg)

        fig_agg.draw()
        time.sleep(inteval)

def vehicle_connect(port, sitl_debug=False, baud=57600, wait_ready=True, timeout=60):
    try:
        if sitl_debug == True:
            vehicles[port] = connect(port, wait_ready=wait_ready)
        else:
            vehicles[port] = connect(port, baud=baud, wait_ready=wait_ready, timeout=timeout)

        # record vehicle team 
        if window["-Radio Red-"].get() == True:
            vehicles_team[port] = "red"
        elif window["-Radio Blue-"].get() == True:
            vehicles_team[port] = "blue"
        elif window["-Radio Green-"].get() == True:
            vehicles_team[port] = "green"
        elif window["-Radio Yellow-"].get() == True:
            vehicles_team[port] = "yellow"
        else:
            logger.error("Vehicle team set error.")
            window["-Status-"].update("Vehicle team set error.")
                    
        # append vehicle location list
        location_x[port] = []
        location_y[port] = []

        # show control tab
        if "-"+port+" Tab-" in window.AllKeysDict:
            print("Visible control tab")
            window["-"+port+" Tab-"].update(visible=True)
            window["-"+port+" Tab-"].select()
        else:
            window["-Tab Group-"].add_tab(sg.Tab(port, control_tab(port), key="-"+port+" Tab-"))
            # IN real hardware autopilot, the DisableGPS checkbox cannot be used.
            window["-"+port+" DisableGPS-"].update(disabled=True)
                    
        logger.info(port+" connected.")
        vehicles_update_threads[port] = threading.Thread(target=update_state_tab, args=(port, update_interval,), daemon=True)
        
        # append active vehicle list
        vehicles_port.append(port)
        # start update thread
        vehicles_update_threads[port].start()

    except:
        # if connection failed
        logger.warning(port+" connection failed.")
        threading.Thread(target=run_pyttsx3, args=(port+"连接失败。",), daemon=True).start()
        window["-Status-"].update(port+" connction failed.")


# mission_thread() for task assignment and target protect
def mission_thread(thisport, inteval):
    # define evaders and pursuers ports
    evader1_port = "tcp:127.0.0.1:5762"
    evader2_port = "tcp:127.0.0.1:5772"
    evader3_port = "tcp:127.0.0.1:5782"
    evader4_port = "tcp:127.0.0.1:5792"
    evader5_port = "tcp:127.0.0.1:5802"
    pursuer1_port = "tcp:127.0.0.1:5812"
    pursuer2_port = "tcp:127.0.0.1:5822"
    pursuer3_port = "tcp:127.0.0.1:5832"
    pursuer4_port = "tcp:127.0.0.1:5842"

    # define evaders and pursuers waiting waypoint
    evader1_waiting_waypoint = LocationGlobalRelative(39.3704854, 115.9121132, 100)
    evader2_waiting_waypoint = LocationGlobalRelative(39.3735210, 115.9151816, 110)
    evader3_waiting_waypoint = LocationGlobalRelative(39.3714309, 115.9185934, 120)
    evader4_waiting_waypoint = LocationGlobalRelative(39.3682791, 115.9158468, 130)
    evader5_waiting_waypoint = LocationGlobalRelative(39.3690754, 115.9209323, 140)

    pursuer1_waiting_waypoint = LocationGlobalRelative(39.3607807, 115.9076071, 105)
    pursuer2_waiting_waypoint = LocationGlobalRelative(39.3599180, 115.9129715, 115)
    pursuer3_waiting_waypoint = LocationGlobalRelative(39.3605484, 115.9205246, 125)
    pursuer4_waiting_waypoint = LocationGlobalRelative(39.3622406, 115.9252453, 135)

    #define evaders attack waypoint
    evader1_attack_waypoint = LocationGlobalRelative(39.3649945, 115.9094095, 105)
    evader2_attack_waypoint = LocationGlobalRelative(39.3647955, 115.9125423, 115)
    evader3_attack_waypoint = LocationGlobalRelative(39.3647955, 115.9163189, 125)
    evader4_attack_waypoint = LocationGlobalRelative(39.3651272, 115.9199238, 135)
    evader5_attack_waypoint = LocationGlobalRelative(39.3660894, 115.9224558, 145)

    # define pursuers task assignment waypoint
    pursuer1_attack_waypoint = evader1_attack_waypoint
    pursuer2_attack_waypoint = evader3_attack_waypoint
    pursuer3_attack_waypoint = evader4_attack_waypoint
    pursuer4_attack_waypoint = evader5_attack_waypoint

    # Flight Mission loop, for "Mission 2" button
    # In Mission Stage 1, pursuer and evaders change alt and seperate to wait takeoff complete
    while window["-Start Mission 2-"].metadata == True and window["-Mission Stage 1-"].get() == True:
        if thisport == evader1_port and vehicles[thisport].location.global_relative_frame.alt > 30:
            vehicles[thisport].mode = VehicleMode("GUIDED")
            vehicles[thisport].simple_goto(evader1_waiting_waypoint)
        elif thisport == evader2_port and vehicles[thisport].location.global_relative_frame.alt > 30:
            vehicles[thisport].mode = VehicleMode("GUIDED")
            vehicles[thisport].simple_goto(evader2_waiting_waypoint)
        elif thisport == evader3_port and vehicles[thisport].location.global_relative_frame.alt > 30:
            vehicles[thisport].mode = VehicleMode("GUIDED")
            vehicles[thisport].simple_goto(evader3_waiting_waypoint)
        elif thisport == evader4_port and vehicles[thisport].location.global_relative_frame.alt > 30:
            vehicles[thisport].mode = VehicleMode("GUIDED")
            vehicles[thisport].simple_goto(evader4_waiting_waypoint)
        elif thisport == evader5_port and vehicles[thisport].location.global_relative_frame.alt >30:
            vehicles[thisport].mode = VehicleMode("GUIDED")
            vehicles[thisport].simple_goto(evader5_waiting_waypoint)
        elif thisport == pursuer1_port and vehicles[thisport].location.global_relative_frame.alt > 30:
            vehicles[thisport].mode = VehicleMode("GUIDED")
            vehicles[thisport].simple_goto(pursuer1_waiting_waypoint)
        elif thisport == pursuer2_port and vehicles[thisport].location.global_relative_frame.alt > 30:
            vehicles[thisport].mode = VehicleMode("GUIDED")
            vehicles[thisport].simple_goto(pursuer2_waiting_waypoint)
        elif thisport == pursuer3_port and vehicles[thisport].location.global_relative_frame.alt > 30:
            vehicles[thisport].mode = VehicleMode("GUIDED")
            vehicles[thisport].simple_goto(pursuer3_waiting_waypoint)
        elif thisport == pursuer4_port and vehicles[thisport].location.global_relative_frame.alt > 30:
            vehicles[thisport].mode = VehicleMode("GUIDED")
            vehicles[thisport].simple_goto(pursuer4_waiting_waypoint)
        
        time.sleep(1)
    
    time.sleep(2)

    # In Mission Stage 2, the team of Evaders begins to attack
    while window["-Start Mission 2-"].metadata == True and window["-Mission Stage 2-"].get() == True:
        if thisport == evader1_port:
            vehicles[thisport].simple_goto(evader1_attack_waypoint)
        elif thisport == evader2_port:
            vehicles[thisport].simple_goto(evader2_attack_waypoint)
        elif thisport == evader3_port:
            vehicles[thisport].simple_goto(evader3_attack_waypoint)
        elif thisport == evader4_port:
            vehicles[thisport].simple_goto(evader4_attack_waypoint)
        elif thisport == evader5_port:
            vehicles[thisport].simple_goto(evader5_attack_waypoint)
        elif thisport == pursuer1_port:
            vehicles[thisport].simple_goto(pursuer1_attack_waypoint)
        elif thisport == pursuer2_port:
            vehicles[thisport].simple_goto(pursuer2_attack_waypoint)
        elif thisport == pursuer3_port:
            vehicles[thisport].simple_goto(pursuer3_attack_waypoint)
        elif thisport == pursuer4_port:
            vehicles[thisport].simple_goto(pursuer4_attack_waypoint)

        time.sleep(1)


# mission_thread() for real flight test
def mission_thread_backup(thisport, inteval):
    # define evader and pursuer ports
    # for sitl
    evader1_port = "tcp:127.0.0.1:5762"
    evader2_port = "tcp:127.0.0.1:5772"
    evader3_port = "tcp:127.0.0.1:5782"
    pursuer_port = "tcp:127.0.0.1:5792"
    # for real vehicles
    # evader1_port = "COM16"
    # evader2_port = "COM34"
    # evader3_port = "COM35"
    # pursuer_port = "COM39"

    pursuer_waiting_waypoint = LocationGlobalRelative(39.3710100, 115.9153962, 80)

    # Flight Mission loop, for "Mission 2" button

    # In Mission Stage 1, pursuer and evaders change alt and seperate to wait takeoff complete
    while window["-Start Mission 2-"].metadata == True and window["-Mission Stage 1-"].get() == True:
        # pursuer_takeoff_waypoint = LocationGlobalRelative(39.3700873, 115.9154177, 80)
        # evader1_takeoff_waypoint = LocationGlobalRelative(39.3696145, 115.9154123, 50)
        # evader2_takeoff_waypoint = LocationGlobalRelative(39.3691376, 115.9154123, 60)
        # evader3_takeoff_waypoint = LocationGlobalRelative(39.3686731, 115.9154123, 70)

        pursuer_takeoff_waypoint = LocationGlobalRelative(39.3703444, 115.9147632, 50)
        evader1_takeoff_waypoint = LocationGlobalRelative(39.3703112, 115.9162223, 60)
        evader2_takeoff_waypoint = LocationGlobalRelative(39.3682957, 115.9147739, 70)
        evader3_takeoff_waypoint = LocationGlobalRelative(39.368304, 115.9162331, 80)

        if thisport == pursuer_port and vehicles[thisport].location.global_relative_frame.alt > 30:
            vehicles[thisport].mode = VehicleMode("GUIDED")
            vehicles[thisport].simple_goto(pursuer_takeoff_waypoint)
        elif thisport == evader1_port and vehicles[thisport].location.global_relative_frame.alt > 30:
            vehicles[thisport].mode = VehicleMode("GUIDED")
            vehicles[thisport].simple_goto(evader1_takeoff_waypoint)
        elif thisport == evader2_port and vehicles[thisport].location.global_relative_frame.alt > 30:
            vehicles[thisport].mode = VehicleMode("GUIDED")
            vehicles[thisport].simple_goto(evader2_takeoff_waypoint)
        elif thisport == evader3_port and vehicles[thisport].location.global_relative_frame.alt > 30:
            vehicles[thisport].mode = VehicleMode("GUIDED")
            vehicles[thisport].simple_goto(evader3_takeoff_waypoint)

        time.sleep(1)

    # 20240114 2v2 combat
    # if thisport == pursuer_port: # pursuer port
    #     cmds = vehicles[thisport].commands
    #     cmds.clear()
    #     cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 39.3703444, 115.9147632, 50))
    #     cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 39.3703444, 115.9147632, 50))                        
    #     cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 39.3703112, 115.9162223, 50))
    #     cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_DO_JUMP, 0, 0, 1, -1, 0, 0, 0, 0, 0))
    #     cmds.upload()
    #     vehicles[thisport].commands.next = 0
    #     vehicles[thisport].mode = VehicleMode("AUTO")
    # elif thisport == evader1_port:
    #     cmds = vehicles[thisport].commands
    #     cmds.clear()
    #     cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 39.3703444, 115.9147632, 60))
    #     cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 39.3703444, 115.9147632, 60))                        
    #     cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 39.3703112, 115.9162223, 60))
    #     cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_DO_JUMP, 0, 0, 1, -1, 0, 0, 0, 0, 0))
    #     cmds.upload()
    #     vehicles[thisport].commands.next = 0
    #     vehicles[thisport].mode = VehicleMode("AUTO")
    # time.sleep(2)
    # # In Mission Stage 2, evader2 and evader 3 begin to chase
    # while window["-Start Mission 2-"].metadata == True and window["-Mission Stage 2-"].get() == True:
        
    #     if thisport == evader2_port:
    #         pursuer_lat = vehicles[pursuer_port].location.global_relative_frame.lat
    #         pursuer_lon = vehicles[pursuer_port].location.global_relative_frame.lon
    #         if get_distance_metres(vehicles[pursuer_port].location.global_relative_frame, vehicles[thisport].location.global_relative_frame) > 90:
    #             vehicles[thisport].simple_goto(LocationGlobalRelative(pursuer_lat, pursuer_lon, 70))
    #         else:
    #             pursuer_bearing = get_bearing_degree(vehicles[pursuer_port].location.global_relative_frame, vehicles[thisport].location.global_relative_frame)
    #             # escape message for evader2
    #             heading_type = 1 # HEADING_TYPE_HEADING
    #             heading_target = pursuer_bearing # deg
    #             heading_roc = 10 # heading rate-of-change
    #             escape_msg = vehicles[thisport].message_factory.command_long_encode(
    #                 0, 0,   # target system, target component
    #                 mavutil.mavlink.MAV_CMD_GUIDED_CHANGE_HEADING,  # command
    #                 0,  # confirmation
    #                 heading_type,     # course-over-ground or raw vehicle heading.
    #                 heading_target, # Target heading. min:0 max:359.99	deg
    #                 heading_roc,    # Maximum centripetal accelearation, ie rate of change, toward new heading. m/s/s
    #                 0, 0, 0, 0  # param 4 - 7
    #             )
    #             vehicles[thisport].mode = VehicleMode("GUIDED")
    #             vehicles[thisport].send_mavlink(escape_msg)
    #     elif thisport == evader3_port:
    #         evader1_lat = vehicles[evader1_port].location.global_relative_frame.lat
    #         evader1_lon = vehicles[evader1_port].location.global_relative_frame.lon
    #         if get_distance_metres(vehicles[evader1_port].location.global_relative_frame, vehicles[thisport].location.global_relative_frame) > 90:
    #             vehicles[thisport].simple_goto(LocationGlobalRelative(evader1_lat, evader1_lon, 80))
    #         else:
    #             evader1_bearing = get_bearing_degree(vehicles[evader1_port].location.global_relative_frame, vehicles[thisport].location.global_relative_frame)
    #             # escape message for evader3
    #             heading_type = 1 # HEADING_TYPE_HEADING
    #             heading_target = evader1_bearing # deg
    #             heading_roc = 10 # heading rate-of-change
    #             escape_msg = vehicles[thisport].message_factory.command_long_encode(
    #                 0, 0,   # target system, target component
    #                 mavutil.mavlink.MAV_CMD_GUIDED_CHANGE_HEADING,  # command
    #                 0,  # confirmation
    #                 heading_type,     # course-over-ground or raw vehicle heading.
    #                 heading_target, # Target heading. min:0 max:359.99	deg
    #                 heading_roc,    # Maximum centripetal accelearation, ie rate of change, toward new heading. m/s/s
    #                 0, 0, 0, 0  # param 4 - 7
    #             )
    #             vehicles[thisport].mode = VehicleMode("GUIDED")
    #             vehicles[thisport].send_mavlink(escape_msg)

    #     time.sleep(1)
    
    # evader 1 and pursuer guided to waypoint
    if thisport == evader1_port: # Evader 1 push waypoints
        cmds = vehicles[thisport].commands
        cmds.clear() # clear old waypoint commands
        cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 39.3703444, 115.9147632, 50))
        cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 39.3703444, 115.9147632, 50))                        
        cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 39.3703112, 115.9162223, 50))
        cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 39.3683040, 115.9162331, 50))
        cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 39.3682957, 115.9147739, 50))
        cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_DO_JUMP, 0, 0, 1, -1, 0, 0, 0, 0, 0))
        cmds.upload()
        vehicles[thisport].commands.next = 0
        vehicles[thisport].mode = VehicleMode("AUTO")
        print("Evader 1 Mode Auto")
    elif thisport == pursuer_port: # Pursuer guided to waiting waypoint
        vehicles[thisport].mode = VehicleMode("GUIDED")
        vehicles[thisport].simple_goto(pursuer_waiting_waypoint)

    time.sleep(2)

    # In Mission Stage 2, the team of Evaders begins to assemble
    while window["-Start Mission 2-"].metadata == True and window["-Mission Stage 2-"].get() == True:
        # clear catchup flag
        window["-CatchUp Flag-"].update(value=False)
        # get evader 1's next wayoint
        next_waypoint_index = vehicles[evader1_port].commands.next
        next_waypoint_lat = vehicles[evader1_port].commands[next_waypoint_index - 1].x
        next_waypoint_lon = vehicles[evader1_port].commands[next_waypoint_index - 1].y
        next_waypoint_alt = vehicles[evader1_port].commands[next_waypoint_index - 1].z
        next_waypoint_location = LocationGlobalRelative(next_waypoint_lat, next_waypoint_lon, next_waypoint_alt)
        # get relative distance
        thisport_distance = get_distance_metres(next_waypoint_location, vehicles[thisport].location.global_relative_frame)
        evader1_distance = get_distance_metres(next_waypoint_location, vehicles[evader1_port].location.global_relative_frame)
        # guided to evader 1's next waypoint according to vehicle number
        if thisport == evader2_port: # Evader 2
            vehicles[thisport].mode = VehicleMode("GUIDED")
            vehicles[thisport].simple_goto(LocationGlobalRelative(next_waypoint_lat,next_waypoint_lon,next_waypoint_alt + 10))
            # airspeed control
            if thisport_distance > evader1_distance:
                vehicles[thisport].airspeed = vehicles[evader1_port].airspeed + 3
            else:
                vehicles[thisport].airspeed = vehicles[evader1_port].airspeed - 3
        elif thisport == evader3_port:
            vehicles[thisport].mode = VehicleMode("GUIDED")
            vehicles[thisport].simple_goto(LocationGlobalRelative(next_waypoint_lat, next_waypoint_lon, next_waypoint_alt + 20))
            # airspeed control
            if thisport_distance > evader1_distance:
                vehicles[thisport].airspeed = vehicles[evader1_port].airspeed + 3
            else:
                vehicles[thisport].airspeed = vehicles[evader1_port].airspeed - 3
        time.sleep(1)
    logger.info("Mission Stage 1 end.")

    # In Mission Stage 3,  the Pursuer begins to chase
    while window["-Start Mission 2-"].metadata == True and window["-Mission Stage 3-"].get() == True:
        logger.info(thisport + ": lat=" + str(vehicles[thisport].location.global_relative_frame.lat) + " lon=" + str(vehicles[thisport].location.global_relative_frame.lon))
        # get evader 1's next waypoint
        next_waypoint_index = vehicles[evader1_port].commands.next
        next_waypoint_lat = vehicles[evader1_port].commands[next_waypoint_index - 1].x
        next_waypoint_lon = vehicles[evader1_port].commands[next_waypoint_index - 1].y
        next_waypoint_alt = vehicles[evader1_port].commands[next_waypoint_index - 1].z
        next_waypoint_location = LocationGlobalRelative(next_waypoint_lat, next_waypoint_lon, next_waypoint_alt)
        thisport_distance = get_distance_metres(next_waypoint_location, vehicles[thisport].location.global_relative_frame)
        evader1_distance = get_distance_metres(next_waypoint_location, vehicles[evader1_port].location.global_relative_frame)     

        # escape message for evaders
        heading_type = 0 # HEADING_TYPE_HEADING
        heading_target = vehicles[thisport].heading - 90 # deg
        if heading_target < 0:
            heading_target = heading_target + 360
        heading_roc = 10 # heading rate-of-change
        escape_msg = vehicles[thisport].message_factory.command_long_encode(
            0, 0,   # target system, target component
            mavutil.mavlink.MAV_CMD_GUIDED_CHANGE_HEADING,  # command
            0,  # confirmation
            heading_type,     # course-over-ground or raw vehicle heading.
            heading_target, # Target heading. min:0 max:359.99	deg
            heading_roc,    # Maximum centripetal accelearation, ie rate of change, toward new heading. m/s/s
            0, 0, 0, 0  # param 4 - 7
        )
           
        # pursuer 
        if thisport == pursuer_port:
            vehicles[thisport].mode = VehicleMode("GUIDED")
            vehicles[thisport].simple_goto(LocationGlobalRelative(next_waypoint_lat, next_waypoint_lon, next_waypoint_alt + 30))
            # vehicles[thisport].simple_goto()
            vehicles[thisport].airspeed = vehicles[evader1_port].airspeed + 3
            catchup_distance = get_distance_metres(vehicles[thisport].location.global_relative_frame, vehicles[evader1_port].location.global_relative_frame)
            print(catchup_distance)
            # if pursuer catch up evaders
            if catchup_distance <= 80:
                # window["-CatchUp Flag-"].update(value=True) # set catchup_flag
                logger.info("Pursuer catch up evaders.")
            else:
                window["-CatchUp Flag-"].update(value=False) # clear catchup_flag

        # evader 2 and 3
        elif thisport == evader2_port:
            if window["-CatchUp Flag-"].get() == False:
                vehicles[thisport].mode = VehicleMode("GUIDED")
                vehicles[thisport].simple_goto(LocationGlobalRelative(next_waypoint_lat,next_waypoint_lon,next_waypoint_alt + 10))
                if thisport_distance > evader1_distance:
                    vehicles[thisport].airspeed = vehicles[evader1_port].airspeed + 3
                else:
                    vehicles[thisport].airspeed = vehicles[evader1_port].airspeed - 3
            else: # pursuer catch up evaders
                vehicles[thisport].mode = VehicleMode("GUIDED")
                vehicles[thisport].send_mavlink(escape_msg)
        
        elif thisport == evader3_port:
            if window["-CatchUp Flag-"].get() == False:
                vehicles[thisport].mode = VehicleMode("GUIDED")
                vehicles[thisport].simple_goto(LocationGlobalRelative(next_waypoint_lat, next_waypoint_lon, next_waypoint_alt + 20))
                if thisport_distance > evader1_distance:
                    vehicles[thisport].airspeed = vehicles[evader1_port].airspeed + 3
                else:
                    vehicles[thisport].airspeed = vehicles[evader1_port].airspeed - 3
            else: # pursuer catch up evaders
                vehicles[thisport].mode = VehicleMode("GUIDED")
                vehicles[thisport].send_mavlink(escape_msg)
        
        time.sleep(1)
    logger.info("Mission Stage 3 end.")

    # In Mission Stage 4, evader 2 and 3 return to formation and pursuer back to waiting waypoint
    while window["-Start Mission 2-"].metadata == True and window["-Mission Stage 4-"].get() == True:
        # get evader 1's next waypoint
        next_waypoint_index = vehicles[evader1_port].commands.next
        next_waypoint_lat = vehicles[evader1_port].commands[next_waypoint_index - 1].x
        next_waypoint_lon = vehicles[evader1_port].commands[next_waypoint_index - 1].y
        next_waypoint_alt = vehicles[evader1_port].commands[next_waypoint_index - 1].z
        next_waypoint_location = LocationGlobalRelative(next_waypoint_lat, next_waypoint_lon, next_waypoint_alt)
        thisport_distance = get_distance_metres(next_waypoint_location, vehicles[thisport].location.global_relative_frame)
        evader1_distance = get_distance_metres(next_waypoint_location, vehicles[evader1_port].location.global_relative_frame) 
        # for pursuer
        if thisport == pursuer_port:
            vehicles[thisport].mode = VehicleMode("GUIDED")
            vehicles[thisport].simple_goto(pursuer_waiting_waypoint)
        # for evader 2 and 3
        elif thisport == evader2_port:
            vehicles[thisport].mode = VehicleMode("GUIDED")
            vehicles[thisport].simple_goto(LocationGlobalRelative(next_waypoint_lat,next_waypoint_lon,next_waypoint_alt + 10))
            print(get_distance_metres(vehicles[thisport].location.global_relative_frame, vehicles[evader1_port].location.global_relative_frame))
            # airspeed control
            if thisport_distance > evader1_distance:
                vehicles[thisport].airspeed = vehicles[evader1_port].airspeed + 3
            else:
                vehicles[thisport].airspeed = vehicles[evader1_port].airspeed - 3
        elif thisport == evader3_port:
            vehicles[thisport].mode = VehicleMode("GUIDED")
            vehicles[thisport].simple_goto(LocationGlobalRelative(next_waypoint_lat,next_waypoint_lon,next_waypoint_alt + 20))
            print(get_distance_metres(vehicles[thisport].location.global_relative_frame, vehicles[evader1_port].location.global_relative_frame))
            # airspeed control
            if thisport_distance > evader1_distance:
                vehicles[thisport].airspeed = vehicles[evader1_port].airspeed + 3
            else:
                vehicles[thisport].airspeed = vehicles[evader1_port].airspeed - 3
        time.sleep(1)
    logger.info("Mission Stage 4 end.")

    # In Mission Stage 5, pursuer and evaders back to takeoff waypoints
    while window["-Start Mission 2-"].metadata == True and window["-Mission Stage 5-"].get() == True:
        pursuer_takeoff_waypoint = LocationGlobalRelative(39.3700873, 115.9154177, 80)
        evader1_takeoff_waypoint = LocationGlobalRelative(39.3696145, 115.9154123, 50)
        evader2_takeoff_waypoint = LocationGlobalRelative(39.3691376, 115.9154123, 60)
        evader3_takeoff_waypoint = LocationGlobalRelative(39.3686731, 115.9154123, 70)

        if thisport == pursuer_port and vehicles[thisport].location.global_relative_frame.alt > 30:
            vehicles[thisport].mode = VehicleMode("GUIDED")
            vehicles[thisport].simple_goto(pursuer_takeoff_waypoint)
        elif thisport == evader1_port and vehicles[thisport].location.global_relative_frame.alt > 30:
            vehicles[thisport].mode = VehicleMode("GUIDED")
            vehicles[thisport].simple_goto(evader1_takeoff_waypoint)
        elif thisport == evader2_port and vehicles[thisport].location.global_relative_frame.alt > 30:
            vehicles[thisport].mode = VehicleMode("GUIDED")
            vehicles[thisport].simple_goto(evader2_takeoff_waypoint)
        elif thisport == evader3_port and vehicles[thisport].location.global_relative_frame.alt > 30:
            vehicles[thisport].mode = VehicleMode("GUIDED")
            vehicles[thisport].simple_goto(evader3_takeoff_waypoint)

        time.sleep(1)
    logger.info("Mission Stage 5 end.")


if __name__ == "__main__":
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

    # Set pyttsx3 engine
    engine = pyttsx3.init()
    engine.setProperty('voice', 'zh')
    # engine.setProperty('rate', 150)
    engine.setProperty('volume', 1)
    
    # Storage Variables
    vehicles = {} # Vehicle dictionary, used by dronekit
    vehicles_team = {} # Vehicle belongs to blue team or red team
    location_x = {} # Vehicle's x location dictionary 
    location_y = {} # Vehicle's y location dictionary
    vehicles_update_threads = {} # update_state_tab() threads
    vehicles_port = []
    update_interval = 0.5
    origin_point = LocationGlobalRelative(39.3696311, 115.9153962, 0) # origin point in matplotlib
    local_ip = socket.gethostbyname(socket.gethostname())

    # Show control window panel
    logger.info("Start control panel.")
    sg.theme("LightGray6")
    window = main_control_panel()
    
    window["-Tab Group-"].add_tab(sg.Tab("blank_port", control_tab("blank_port"), key="-Blank Tab-"))
    window["-Blank Tab-"].Update(visible=False)
    window["-Start Mission 1-"].metadata = False
    window["-Start Mission 2-"].metadata = False
    # Initialize 
    canvas_elem = window['-CANVAS-']
    canvas = canvas_elem.TKCanvas

    # draw the initial plot in the window
    fig = Figure(figsize=(6,6))
    ax = fig.add_subplot(111)
    ax.set_xlabel("X axis")
    ax.set_ylabel("Y axis")
    ax.grid(visible=True, which="both")
    ax.axis("equal")
    fig_agg = draw_figure(canvas, fig)
    threading.Thread(target=update_animate, args=(update_interval,), daemon=True).start()

    # Event process loop
    while True:
        event, values = window.read()
        # print(event, values)
        
        # Event: Close button pushed.
        if event == sg.WINDOW_CLOSE_ATTEMPTED_EVENT or event == "Exit":
            ch = sg.popup_ok_cancel("确定断开所有连接并退出吗？", title="Confirm Exit?", keep_on_top=True)
            if ch == "OK":
                logger.info("Control panel closed.")
                break
        
        # Event: SITL combo box checked.
        if event == "-SITL Debug-" and values["-SITL Debug-"] == True:
            logger.info("SITL debugging, use TCP connection.")
            tcp_device = values["-TCP Port-"] #["tcp:127.0.0.1:5762", "tcp:127.0.0.1:5772"]
            window["-Ports-"].update(value=tcp_device)
            window["-Status-"].update("SITL debugging, use TCP connection.")
            threading.Thread(target=run_pyttsx3, args=("SITL仿真模式。",), daemon=True).start()
        


        # Event: TCP port inputtext changed
        if event == "-TCP Port-" and values["-SITL Debug-"] == True:
            tcp_device = values["-TCP Port-"]
            window["-Ports-"].update(value=tcp_device)

        # Event: Refresh button pressed.
        if event == "-Refresh Ports-" and values["-SITL Debug-"] == False:
            logger.info("Refresh available serial ports.")
            ports_list = list(serial.tools.list_ports.comports())
            if len(ports_list) <= 0:
                logger.info("No serial ports.")
                window["-Ports-"].update(values=[])
                window["-Status-"].update("No serial ports.")
                threading.Thread(target=run_pyttsx3, args=("错误，没有串口设备。",), daemon=True).start()
            else:
                # check available serial ports
                available_ports = []
                for comport in ports_list:
                    if comport.manufacturer == "Silicon Laboratories" or comport.manufacturer == "Silicon Labs":
                        available_ports.append(comport.device)
                if len(available_ports) <= 0:
                    window["-Ports-"].update(values=[])
                    window["-Status-"].update("No available serial ports.")  
                    threading.Thread(target=run_pyttsx3, args=("错误，未找到数传串口设备。",), daemon=True).start()    
                else:
                    window["-Ports-"].update(values=available_ports)
                    window["-Ports-"].update(value=available_ports[0])
                    window["-Status-"].update("Available serial ports refreshed.")
                    threading.Thread(target=run_pyttsx3, args=("串口设备已刷新。",), daemon=True).start()
        
        # Event: Connect button pressed.
        if event == "-Connect-":
            port = values["-Ports-"]
            print(port)
            if port == "":
                window["-Status-"].update("No port selected.")
                threading.Thread(target=run_pyttsx3, args=("错误，未选择串口设备。",), daemon=True).start()
            else:
                window["-Status-"].update(port + " selected, connecting...")
                threading.Thread(target=run_pyttsx3, args=("正在连接"+port,), daemon=True).start()

                threading.Thread(target=vehicle_connect, args=(port, window["-SITL Debug-"].get(), 57600, True, 60,), daemon=True).start()
        # ------------------------------------------------------------------
        # Event in control tab, must find target_port.
        # Event: Arm/Disarm button.
        if event.find("ArmDisarm") >= 0:
            for port in vehicles_port:
                if event.find(port) >= 0:
                    target_port = port
            if vehicles[target_port].armed == True and values["-"+target_port+" Force-"] == False:
                vehicles[target_port].armed = False
                logger.info(target_port+" attempt disarm.")
            elif vehicles[target_port].armed == False and values["-"+target_port+" Force-"] == False:
                vehicles[target_port].armed = True
                logger.info(target_port+" attempt arm.")
            elif vehicles[target_port].armed == True and values["-"+target_port+" Force-"] == True:
                # create the MAV_CMD_COMPONENT_ARM_DISARM command using command_long_encode()
                msg = vehicles[target_port].message_factory.command_long_encode(
                    0, 0,    # target system, target component
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, #command
                    0, #confirmation
                    0,    # param 1, disarm
                    21196,    # param 2, 21196: force arming/disarming (e.g. allow arming to override preflight checks and disarming in flight)
                    0, 0, 0, 0, 0) # param 3 ~ 7 not used
                # send command to vehicle
                vehicles[target_port].send_mavlink(msg)
            elif vehicles[target_port].armed == False and values["-"+target_port+" Force-"] == True:
                # create the MAV_CMD_COMPONENT_ARM_DISARM command using command_long_encode()
                msg = vehicles[target_port].message_factory.command_long_encode(
                    0, 0,    # target system, target component
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, #command
                    0, #confirmation
                    1,    # param 1, arm
                    2989,    # param 2, 21196: force arming/disarming (e.g. allow arming to override preflight checks and disarming in flight)
                    0, 0, 0, 0, 0) # param 3 ~ 7 not used
                # send command to vehicle
                vehicles[target_port].send_mavlink(msg)

        # Event: Set mode button pressed.
        if event.find("SetMode") >= 0:
            for port in vehicles_port:
                if event.find(port) >= 0:
                    target_port = port
            target_mode = values["-"+target_port+" ModeCombo-"]
            if target_mode == vehicles[target_port].mode.name:
                window["-Status-"].update("Already in "+target_mode+" mode.")
                threading.Thread(target=run_pyttsx3, args=(target_mode,), daemon=True).start()
            else:
                vehicles[target_port].mode = VehicleMode(target_mode)
                window["-Status-"].update("Set mode to "+target_mode+".")
                threading.Thread(target=run_pyttsx3, args=(target_mode,), daemon=True).start()

        # Event: Airspeed Button pressed.
        if event.find("AirspeedButton") >= 0:
            for port in vehicles_port:
                if event.find(port) >= 0:
                    target_port = port
            target_airspeed = float(values["-"+target_port+" AirspeedInput-"])
            vehicles[target_port].airspeed = target_airspeed
            window["-Status-"].update("Set target airspeed "+str(target_airspeed)+"m/s.")
            logger.info(target_port+" target airspeed "+str(target_airspeed)+"m/s.")
        
        # Event: Disable GPS Checkbox. USE SIM_GPS_DISABLE parameter.
        # NOTEthat NOT woring in Cube Orange hardware 
        if event.find("DisableGPS") >= 0:
            for port in vehicles_port:
                if event.find(port) >= 0:
                    target_port = port
            if values[event] == True:
                while vehicles[target_port].parameters["SIM_GPS_DISABLE"] == False:
                    vehicles[target_port].parameters["SIM_GPS_DISABLE"] = True
                logger.info(target_port+" GPS disabled.")
                window["-Status-"].update("Disable GPS.")
            else:
                while vehicles[target_port].parameters["SIM_GPS_DISABLE"] == True:
                    vehicles[target_port].parameters["SIM_GPS_DISABLE"] = False
                logger.info(target_port+" GPS enabled.")
                window["-Status-"].update("Enable GPS.")
        
        # Event: Disable RC Failsafe Checkbox. USE THR_FAILSAFE parameter.
        if event.find("DisableRCFailsafe") >= 0:
            for port in vehicles_port:
                if event.find(port) >= 0:
                    target_port = port
            if values[event] == True:
                while vehicles[target_port].parameters["THR_FAILSAFE"] == 1:
                    vehicles[target_port].parameters["THR_FAILSAFE"] = 0
                logger.info(target_port+" RC failsafe disabled.")
                window["-Status-"].update("RC failsafe disabled.")
            else:
                while vehicles[target_port].parameters["THR_FAILSAFE"] == 0:
                    vehicles[target_port].parameters["THR_FAILSAFE"] = 1
                logger.info(target_port+" RC failsafe enabled.")
                window["-Status-"].update("RC failsafe enabled.")
        
        # Event: Disable Serial 5 Protocol Checkbox. USE SERIAL5_PROTOCOL parameter.
        if event.find("DisableSerial5Protocol") >= 0:
            for port in vehicles_port:
                if event.find(port) >= 0:
                    target_port = port
            if values[event] == True:
                while vehicles[target_port].parameters["SERIAL5_PROTOCOL"] != -1:
                    vehicles[target_port].parameters["SERIAL5_PROTOCOL"] = -1
                logger.info(target_port+" Serial 5 Protocol disabled.")
                window["-Status-"].update("Serial 5 Protocol disabled.")

        # Event Land button pressed.
        # NOTICE: Use GlobalControl LAND command.
        if event.find("Separate Land") >= 0:
            for port in vehicles_port:
                if event.find(port) >= 0:
                    target_port = port
            cmds = vehicles[target_port].commands
            cmds.clear() # clear old waypoint commands
            cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_DO_LAND_START, 0, 0, 0, 0, 0, 0, 39.3654715, 115.916217, 50))
            # The first waypoint command is written twice to prevent it from being executed
            cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_DO_LAND_START, 0, 0, 0, 0, 0, 0, 39.3654715, 115.916217, 50))
            cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 39.3654632, 115.9154123, 50))
            cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 39.3669054, 115.9153989, 30))
            cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 39.3680635, 115.9154069, 15))
            cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 39.3691086, 115.9154123, 0))
            cmds.upload()
            vehicles[target_port].commands.next = 0
            vehicles[target_port].mode = VehicleMode("AUTO")
            
        # Event: Reboot button pressed.
        # NOTICE: NOT Working in SITL Simulation, USE Carefully!!!
        # NOTICE: Working in real hardware autopilot, USE CAREFULLY!!! 
        if event.find("Reboot") >= 0:
            for port in vehicles_port:
                if event.find(port) >= 0:
                    target_port = port
            vehicles[target_port].reboot()

        # Event: Disconnect button pressed.
        if event.find("Disconnect") >= 0:
            for port in vehicles_port:
                if event.find(port) >= 0:
                    target_port = port
            # MUST kill update_state_tab() thread first!
            vehicles_port.remove(target_port)
            while vehicles_update_threads[target_port].is_alive():
                time.sleep(0.1)
            # hide tab rather than delete it
            window["-"+target_port+" Tab-"].Update(visible=False)
            vehicles[target_port].close()
            del vehicles[target_port] 
            del vehicles_team[target_port]
            del vehicles_update_threads[target_port]
            logger.info("Disconnect "+target_port+".")
            threading.Thread(target=run_pyttsx3, args=(target_port+" 断开连接。",), daemon=True).start()
        # ------------------------------------------------------------------
        # Event in Global Control Frame, must select all available vehicles first!
        # Event: Global control ARM
        if event == "-GlobalControl ARM-":
            for port in vehicles_port:
                vehicles[port].armed = True
                logger.info(port+" attempt arm.")
            window["-Status-"].update("Global control arm.")

        # Event: Global control Disarm
        if event == "-GlobalControl DISARM-":
            for port in vehicles_port:
                vehicles[port].armed = False
                logger.info(port+" attempt disarm.")
            window["-Status-"].update("Global control disarm.")
        
        # Event: Execute landing sequence
        if event == "-GlobalControl LANDSEQ-":
            for port in vehicles_port:
                cmds = vehicles[port].commands
                cmds.clear() # clear old waypoint commands
                cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_DO_LAND_START, 0, 0, 0, 0, 0, 0, 39.3654715, 115.916217, 50))
                # The first waypoint command is written twice to prevent it from being executed
                cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_DO_LAND_START, 0, 0, 0, 0, 0, 0, 39.3654715, 115.916217, 50))
                cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 39.3654632, 115.9154123, 50))
                cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 39.3669054, 115.9153989, 30))
                cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 39.3680635, 115.9154069, 15))
                cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 39.3691086, 115.9154123, 0))
                cmds.upload()
                vehicles[port].commands.next = 0
                vehicles[port].mode = VehicleMode("AUTO")
        
        # Event: Motor Emergency Stop, USE Carefully
        if event == "-GlobalControl EMERGENCYSTOP-":
            for port in vehicles_port:
                vehicles[port].channels.overrides['7'] = 1900

        # Event: Global control TAKEOFF mode
        if event == "-GlobalControl TAKEOFF-":
            for port in vehicles_port:
                vehicles[port].mode = VehicleMode("TAKEOFF")
                logger.info(port+" set to TAKEOFF mode.")
            window["-Status-"].update("Global control set to TAKEOFF mode.")

        # Event: Global control FBWA mode
        if event == "-GlobalControl FBWA-":
            for port in vehicles_port:
                vehicles[port].mode = VehicleMode("FBWA")
                logger.info(port+" set to FBWA mode.")
            window["-Status-"].update("Global control set to FBWA mode.")

        # Event: Global control GUIDED mode
        if event == "-GlobalControl GUIDED-":
            for port in vehicles_port:
                vehicles[port].mode = VehicleMode("GUIDED")
                logger.info(port+" set to GUIDED mode.")
            window["-Status-"].update("Global control set to GUIDED mode.")
        
        # Event: Global control RTL mode
        if event == "-GlobalControl RTL-":
            for port in vehicles_port:
                vehicles[port].mode = VehicleMode("RTL")
                logger.info(port+" set to RTL mode.")
            window["-Status-"].update("Global control set to TAKEOFF mode.")
        
        # Event: Global control MANUAL mode
        if event == "-GlobalControl MANUAL-":
            for port in vehicles_port:
                vehicles[port].mode = VehicleMode("MANUAL")
                logger.info(port+" set to MANUAL mode.")
            window["-Status-"].update("Global control set to MANUAL mode.")
        
        # Event: Global control AUTO mode
        if event == "-GlobalControl AUTO-":
            for port in vehicles_port:
                vehicles[port].mode = VehicleMode("AUTO")
                logger.info(port+" set to AUTO mode.")
            window["-Status-"].update("Global control set to AUTO mode.")

        # Event: Global control QLOITER mode
        if event == "-GlobalControl QLOITER-":
            for port in vehicles_port:
                vehicles[port].mode = VehicleMode("QLOITER")
                logger.info(port+" set to QLOITER mode.")
            window["-Status-"].update("Global control set to QLOITER mode.")
        
        # Event: Global control QLAND mode
        if event == "-GlobalControl QLOITER-":
            for port in vehicles_port:
                vehicles[port].mode = VehicleMode("QLOITER")
                logger.info(port+" set to QLOITER mode.")
            window["-Status-"].update("Global control set to QLOITER mode.")

        # Event: Start Mission 1 button pressed, perform the pursuit mission procedure
        # Disabled
        if event == "-Start Mission 1-":
            if window["-Start Mission 1-"].metadata == False:
                window["-Start Mission 1-"].metadata = True
            window["-Status-"].update("Start mission 1.")
            logger.info("Start mission 1.")
            for port in vehicles_port:
                print(port)

        # Event: Abort Mission 1 button pressed, all vehicles set to RTL mode
        if event == "-Abort Mission 1-":
            if window['-Start Mission 1-'].metadata == True:
                for port in vehicles_port:
                    vehicles[port].mode = VehicleMode("RTL")
            window['-Start Mission 1-'].metadata = False
            logger.info("Abort Mission 1.")
            window["-Status-"].update("Abort Mission 1.")

        # Event: Start Mission 2 button pressed, run mission_thread()
        if event == "-Start Mission 2-":
            vehicles_mission_threads = {} # mission_thread() threads dictionary
            vehicles_reached_rally_point = {} # vehicle have reached the rally point dictionary
            if window["-Start Mission 2-"].metadata == False:
                window["-Start Mission 2-"].metadata = True
                for port in vehicles_port:
                    vehicles_mission_threads[port] = threading.Thread(target=mission_thread, args=(port, update_interval,), daemon=True)
                    vehicles_reached_rally_point[port] = False
                    vehicles_mission_threads[port].start()
                logger.info("Start Mission 2.")
                window["-Status-"].update("Start Mission 2.")
            else:
                logger.info("Mission 2 has already in progress.")
                window["-Status-"].update("Mission 2 has already in progress.")
                run_pyttsx3("任务2正在执行中。")
        
        # Event: Abort Mission 2 button, wait for all vehicles_mission_threads stopped
        if event == "-Abort Mission 2-":
            if window["-Start Mission 2-"].metadata == True:
                window["-Start Mission 2-"].metadata = False
                for port in vehicles_port:
                    vehicles_mission_threads[port].join()
                logger.info("Abort Mission 2.")
                window["-Status-"].update("Abort Mission 2.")
            else:
                logger.info("Mission 2 has been completed.")
                window["-Status-"].update("Mission 2 has been completed.")
                run_pyttsx3("任务2已完成。")
        # ------------------------------------------------------------------
        # Event: Version button pressed.
        if event == 'Version':
            sg.popup_scrolled(sg.get_versions(), non_blocking=True)
        
        # Event: Test button pressed.
        if event == "Test":
            port = "tcp:127.0.0.1:5762"
            try:
                print(vehicles[port].parameters["SIM_GPS"])
            except:
                window["-Status-"].update("key error")
    
    window.close()
