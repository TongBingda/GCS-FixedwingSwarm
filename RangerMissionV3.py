"""
FMS Ranger 1800mm Mission, Test For Automatic Takeoff, Landing And Waypoint Mission
2023.06.28 Start building UAV ground control stations with pysimplegui 
and abandon the solution of directly using matplotlib to plot UAV trajectory
Version 20230628
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
import logging, threading, serial, serial.tools.list_ports, pyttsx3, datetime, time, math, sys, os, random

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
        [
            sg.Button("Start Mission 1", key="-Start Mission 1-"), 
            sg.Button("Abort Mission 1", key="-Abort Mission 1-"),
            sg.Button("Start Mission 2", key="-Start Mission 2-"),
            sg.Button("Abort Mission 2", key="-Abort Mission 2-")
        ]
    ]
    # left layout: control panel
    layout_l = [
        [sg.Button("Refresh Ports", key="-Refresh Ports-"), sg.Text("Ports:", justification="left"), sg.Combo([], size=(18,1), key="-Ports-"), sg.Button("Connect", key="-Connect-")],
        [sg.Text("Vehicle Team:"), sg.Radio("Red", "Team Radio", default=True, key="-Radio Red-"), sg.Radio("Blue", "Team Radio", default=False, key="-Radio Blue-")],
        [sg.TabGroup([], key="-Tab Group-")],
        [sg.Frame("Global Control", global_control)]
    ]
    # right layout: display panel
    layout_r = [
        [sg.Canvas(size=(800, 800), key="-CANVAS-")]
    ]
    # main layout
    layout = [
        [sg.Text("         固定翼无人机集群地面站           ", font="黑体 24"), sg.Text("Port:"), sg.Input(default_text=5762, size=(4,1), key="-TCP Port-", enable_events=True), sg.Checkbox("SITL Simulation", enable_events=True, key="-SITL Debug-"), sg.Exit()],
        [sg.Column(layout_l, vertical_alignment="top"), sg.Column(layout_r)],
        # [sg.Column(layout_l, justification="left", element_justification="left", vertical_alignment="top"), sg.Column(layout_r, justification="right")],
        [sg.Button('Version'), sg.StatusBar(text="Information output here.", size=(50,1), key="-Status-")]
    ]
    
    window = sg.Window("UAV Swarm Control Panel", layout, finalize=True, keep_on_top=False)

    return window

def control_tab(thisport):
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
                sg.Combo(["MANUAL", "FBWA", "GUIDED", "RTL", "AUTO", "CRUISE", "TAKEOFF"], default_value="GUIDED", size=(10,1), key="-"+thisport+" ModeCombo-"), 
                sg.Button("Set Mode", key="-"+thisport+" SetMode-")
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
                sg.Text("CH1:"), sg.ProgressBar(1900, size=(6,20), key="-"+thisport+" CH1-"),
                sg.Text("CH2:"), sg.ProgressBar(1900, size=(6,20), key="-"+thisport+" CH2-"),
                sg.Text("CH3:"), sg.ProgressBar(1900, size=(6,20), key="-"+thisport+" CH3-"),
                sg.Text("CH4:"), sg.ProgressBar(1900, size=(6,20), key="-"+thisport+" CH4-")
            ],
            # new column 10
            [
                sg.Text("CH5:"), sg.ProgressBar(1900, size=(6,20), key="-"+thisport+" CH5-"),
                sg.Text("CH6:"), sg.ProgressBar(1900, size=(6,20), key="-"+thisport+" CH6-"),
                sg.Text("CH7:"), sg.ProgressBar(1900, size=(6,20), key="-"+thisport+" CH7-"),
                sg.Text("CH8:"), sg.ProgressBar(1900, size=(6,20), key="-"+thisport+" CH8-")
            ],
            # new column 11
            [
                sg.Text("Out1:"), sg.ProgressBar(1900, size=(6,20), key="-"+thisport+" OUT1-"),
                sg.Text("Out2:"), sg.ProgressBar(1900, size=(6,20), key="-"+thisport+" OUT2-"),
                sg.Text("Out3:"), sg.ProgressBar(1900, size=(6,20), key="-"+thisport+" OUT3-"),
                sg.Text("Out4:"), sg.ProgressBar(1900, size=(6,20), key="-"+thisport+" OUT4-")
            ],
            # new column 12
            [
                sg.Text("Out5:"),sg.ProgressBar(1900, size=(6,20), key="-"+thisport+" OUT5-"),
                sg.Text("Out6:"),sg.ProgressBar(1900, size=(6,20), key="-"+thisport+" OUT6-"),
                sg.Text("Out7:"),sg.ProgressBar(1900, size=(6,20), key="-"+thisport+" OUT7-"),
                sg.Text("Out8:"),sg.ProgressBar(1900, size=(6,20), key="-"+thisport+" OUT8-"),
            ],
            # new column 13
            [
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

def update_state_tab(thisport, inteval=1, max_num=100):
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
        # layout_l column 11
        window["-"+thisport+" OUT1-"].update(current_count=vehicles[thisport].servos.servo_raw["1"])
        window["-"+thisport+" OUT2-"].update(current_count=vehicles[thisport].servos.servo_raw["2"])
        window["-"+thisport+" OUT3-"].update(current_count=vehicles[thisport].servos.servo_raw["3"])
        window["-"+thisport+" OUT4-"].update(current_count=vehicles[thisport].servos.servo_raw["4"])
        # layout_l column 12
        window["-"+thisport+" OUT5-"].update(current_count=vehicles[thisport].servos.servo_raw["5"])
        window["-"+thisport+" OUT6-"].update(current_count=vehicles[thisport].servos.servo_raw["6"])
        window["-"+thisport+" OUT7-"].update(current_count=vehicles[thisport].servos.servo_raw["7"])
        window["-"+thisport+" OUT8-"].update(current_count=vehicles[thisport].servos.servo_raw["8"])
        # update vehicle location dictionary
        this_x, this_y, this_z = get_grid_location(origin_point, vehicles[thisport].location.global_relative_frame)
        location_x[thisport].append(this_x)
        location_y[thisport].append(this_y)
        if len(location_x[thisport]) > max_num:
            location_x[thisport].pop(0)
            location_y[thisport].pop(0)
        
        if window["-"+thisport+" LivePlot-"].get() == True:
            # call update_animate() function, draw trajectory
            update_animate()

        # log vehicle's information
        # logger.info(thisport+": "+vehicles[thisport].location.global_relative_frame)
        time.sleep(inteval)

def update_animate():
    ax.cla()
    ax.grid()
    ax.set_xlabel("X axis(m)")
    ax.set_ylabel("Y axis(m)")
    ax.grid("equal")
    for thisport in vehicles_port:
        if window["-"+thisport+" LivePlot-"].get() == True:
            ax.plot(location_x[thisport], location_y[thisport], color=vehicles_team[thisport], linewidth=2)
            ax.scatter(location_x[thisport][-1], location_y[thisport][-1], color=vehicles_team[thisport], s=80,marker="$"+str(vehicles_port.index(port)+1)+"$")
    fig_agg.draw()

def mission_thread(port, inteval):
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
    origin_point = LocationGlobalRelative(39.3696311, 115.9153962, 0)

    # Show control window panel
    logger.info("Start control panel.")
    window = main_control_panel()

    window["-Tab Group-"].add_tab(sg.Tab("blank_port", control_tab("blank_port"), key="-Blank Tab-"))
    window["-Blank Tab-"].Update(visible=False)
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

    # Event process loop
    while True:
        event, values = window.read()
        # print(event, values)
        
        # Event: Close button pushed.
        if event == sg.WIN_CLOSED or event == "Exit":
            ch = sg.popup_ok_cancel("确定断开所有连接并退出吗？", title="Confirm Exit?", keep_on_top=True)
            if ch == "OK":
                logger.info("Control panel closed.")
                break
        
        # Event: SITL combo box checked.
        if event == "-SITL Debug-" and values["-SITL Debug-"] == True:
            logger.info("SITL debugging, use TCP connection.")
            tcp_device = "tcp:127.0.0.1:"+values["-TCP Port-"] #["tcp:127.0.0.1:5762", "tcp:127.0.0.1:5772"]
            window["-Ports-"].update(value=tcp_device)
            window["-Status-"].update("SITL debugging, use TCP connection.")
            threading.Thread(target=run_pyttsx3, args=("SITL仿真模式。",), daemon=True).start()
        
        # Event: TCP port inputtext changed
        if event == "-TCP Port-" and values["-SITL Debug-"] == True:
            tcp_device = "tcp:127.0.0.1:"+values["-TCP Port-"]
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

                if values["-SITL Debug-"] == False: # USE serial port
                    # Use device ports to differentiate vehicles
                    try:
                        vehicles[port] = connect(port, baud=57600, wait_ready=True)
                    except:
                        # if connection failed
                        logger.warning(port+" connection failed.")
                        threading.Thread(target=run_pyttsx3, args=(port+"连接失败。",), daemon=True).start()
                        window["-Status-"].update(port+" connction failed.")
                        continue

                    # append active vehicle list
                    vehicles_port.append(port)
                    # record vehicle team 
                    if values["-Radio Red-"] == True:
                        vehicles_team[port] = "red"
                    elif values["-Radio Blue-"] == True:
                        vehicles_team[port] = "blue"
                    else:
                        logger.error("Vehicle team set error.")
                        window["-Status-"].update("Vehicle team set error.")
                        continue
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
                    
                    logger.info(port+" connected.")
                    vehicles_update_threads[port] = threading.Thread(target=update_state_tab, args=(port, update_interval,), daemon=True)
                    vehicles_update_threads[port].start()
                else: # USE TCP port
                    try:
                        vehicles[port] = connect(port, wait_ready=True)
                    except:
                        # if connection failed
                        logger.warning(port+" connection failed.")
                        threading.Thread(target=run_pyttsx3, args=(port+"连接失败。",), daemon=True).start()
                        window["-Status-"].update(port+" connction failed.")
                        continue

                    # append active vehicle list
                    vehicles_port.append(port)
                    # record vehicle team 
                    if values["-Radio Red-"] == True:
                        vehicles_team[port] = "red"
                    elif values["-Radio Blue-"] == True:
                        vehicles_team[port] = "blue"
                    else:
                        logger.error("Vehicle team set error.")
                        window["-Status-"].update("Vehicle team set error.")
                        continue
                    # append vehicle location list
                    location_x[port] = []
                    location_y[port] = []

                    # show control tab
                    if "-"+port+" Tab-" in window.AllKeysDict:
                        window["-"+port+" Tab-"].update(visible=True)
                        window["-"+port+" Tab-"].select()
                    else:
                        window["-Tab Group-"].add_tab(sg.Tab(port, control_tab(port), key="-"+port+" Tab-"))

                    vehicles_update_threads[port] = threading.Thread(target=update_state_tab, args=(port, update_interval,), daemon=True)
                    vehicles_update_threads[port].start()
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

        # Event: Reboot button pressed.
        # NOTICE: NOT Working in SITL Simulation, USE Carefully!!!
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

        if event == "-GlobalControl GUIDED-":
            for port in vehicles_port:
                vehicles[port].mode = VehicleMode("GUIDED")
                logger.info(port+" set to GUIDED mode.")
            window["-Status-"].update("Global control set to GUIDED mode.")
        
        if event == "-GlobalControl FBWA-":
            for port in vehicles_port:
                vehicles[port].mode = VehicleMode("FBWA")
                logger.info(port+" set to FBWA mode.")
            window["-Status-"].update("Global control set to FBWA mode.")
        
        if event == "-GlobalControl AUTO-":
            for port in vehicles_port:
                vehicles[port].mode = VehicleMode("AUTO")
                logger.info(port+" set to AUTO mode.")
            window["-Status-"].update("Global control set to AUTO mode.")

        if event == "-GlobalControl TAKEOFF-":
            for port in vehicles_port:
                vehicles[port].mode = VehicleMode("TAKEOFF")
                logger.info(port+" set to TAKEOFF mode.")
            window["-Status-"].update("Global control set to TAKEOFF mode.")
        
        if event == "-GlobalControl RTL-":
            for port in vehicles_port:
                vehicles[port].mode = VehicleMode("RTL")
                logger.info(port+" set to RTL mode.")
            window["-Status-"].update("Global control set to TAKEOFF mode.")

        if event == "-Start Mission 1-":
            for port in vehicles_port:
                cmds = vehicles[port].commands
                cmds.clear()
                #Add MAV_CMD_NAV_TAKEOFF command. This is ignored if the vehicle is already in the air.
                # cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, 10))
                cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 39.3680967, 115.9100318, 50))
                cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 39.3689095, 115.9199882, 80))
                cmds.upload()
                vehicles[port].commands.next = 0
                vehicles[port].mode = VehicleMode("AUTO")
        # ------------------------------------------------------------------
        # Event: Version button pressed.
        if event == 'Version':
            sg.popup_scrolled(sg.get_versions(), non_blocking=True)
    
    window.close()

"""
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
"""