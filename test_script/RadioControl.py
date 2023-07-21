import serial
import serial.tools.list_ports
import time
if __name__ == "__main__":
    ports_list = list(serial.tools.list_ports.comports())
    if len(ports_list) <= 0:
        print("No serial ports.")
    else:
        print("Available serial ports:")
        for comport in ports_list:
            print(list(comport)[0], list(comport)[1])

    receive = serial.Serial("COM11", 57600, timeout=10)
    send = serial.Serial("COM3", 57600, timeout=10)
    while True:
        header = receive.read(1)
        while header != b'\x0f':
            header = receive.read(1)
        data = receive.read(34)
        packet = header + data
        send.write(packet)
