import time
from dronekit import connect

if __name__ == "__main__":
    vehicle = connect("tcp:127.0.0.1:5763", wait_ready=True)
    print("Channel values from RC Tx:", vehicle.channels)

    # Access channels individually
    print("Read channels individually:")
    print(" Ch1: %s" % vehicle.channels['1'])
    print(" Ch2: %s" % vehicle.channels['2'])
    print(" Ch3: %s" % vehicle.channels['3'])
    print(" Ch4: %s" % vehicle.channels['4'])
    print(" Ch5: %s" % vehicle.channels['5'])
    print(" Ch6: %s" % vehicle.channels['6'])
    print(" Ch7: %s" % vehicle.channels['7'])
    print(" Ch8: %s" % vehicle.channels['8'])
    print("Number of channels: %s" % len(vehicle.channels))

    # Override channels
    # while True:
    #     print("Set Ch3 override to 200 (indexing syntax)")
    #     vehicle.channels.overrides['3'] = 1800
    #     print(" Channel overrides: %s" % vehicle.channels.overrides) 
    #     time.sleep(1)
    #     if vehicle.mode.name != "GUIDED":
    #         break 

    # vehicle.channels.overrides['2'] = None
    print("\nClose vehicle object")
    vehicle.close()