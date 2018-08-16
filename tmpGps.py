import serial
import time

with serial.Serial('/dev/ttyACM0', baudrate=115200) as gpsPort:
    while True:
        #print gpsPort.inWaiting()
        print gpsPort.readline()
