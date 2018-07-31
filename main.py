# coding=utf-8
# Tested in Python 2.7.13.

from Adafruit_BNO055 import BNO055 as move_sensor
import time
import cv2
import plan_route
import parse
import serial
import Tkinter as tk
import os
import json
import math


def calibrateMoveSensor(bno):
    cal = bno.get_calibration_status()
    while not isCalibrated(cal):
        time.sleep(0.5)
        print calibrationMsg(cal)
        cal = bno.get_calibration_status()


def subtractAngles(angle1, angle2):
    """
    The angles are between 0 and 2 pi.  It takes the difference and produces
    an angle between 0 and 2 pi.
    """
    twoPi = 2 * math.pi
    diff = angle1 - angle2
    if diff > twoPi:
        return diff - twoPi
    if diff < 0:
        return diff + twoPi


def calibrationMsg(calDat):
    sys, gyro, accel, mag = calDat
    ending = " not yet calibrated"
    if sys < 3:
        return "sys" + ending
    if gyro < 3:
        return "gyro" + ending
    if accel < 3:
        return "accel" + ending
    if mag < 3:
        return "mag" + ending


def setupMoveSensor():
    handle, err = connectToMoveSensor()
    if err is not None:
        return None, err
    # calibrateMoveSensor(handle)
    return handle, None


def isCalibrated(calDat):
    sys, gyro, accel, mag = calDat
    return sys == 3 and gyro == 3 and accel == 3 and mag == 3


def connectToMoveSensor():
    handle = move_sensor.BNO055(serial_port='/dev/serial0', rst=18)
    if not handle.begin():
        return (None, "Could not connect to movement sensor.")
    return (handle, None)


def readMoveSensor(bno):
    heading, roll, pitch = bno.read_euler()
    x, y, z = bno.read_accelerometer()
    return {
        'heading': heading,
        'roll': roll,
        'pitch': pitch,
        'accelx': x,
        'accely': y,
        'accelz': z}


def testMoveSensor():
    handle, err = connectToMoveSensor()
    if err is not None:
        print err
        return
    while True:
        print readMoveSensor(handle)


def connectToCam(camNum):
    handle = cv2.VideoCapture(camNum)
    if not handle.isOpened():
        return None, "Could not connect to webcam {}".format(camNum)
    return handle, None


def takePhoto(camHandle):
    ok, photo = camHandle.read()
    if not ok:
        return None, "Could not take photo."
    return photo, None


def toBlackAndWhite(im):
    return cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)


def readGps(gpsSerialPort):
    """
    It tries several times to read the GPS receiver and get a valid
    reading.
    """
    reading = {
        'speed': None,
        'speedError': 'No speed reading available.',
        'position': None,
        'positionError': 'No position reading available.'}
    waiting = gpsSerialPort.inWaiting()
    if waiting < 500:
        return None, 'GPS buffer not full enough'
    for _ in range(14):
        raw = gpsSerialPort.readline()
        if reading['speedError'] is not None:
            reading['speed'], reading['speedError'] = (
                parse_gps_speed_reading(raw))
        if reading['positionError'] is not None:
            reading['position'], reading['positionError'] = (
                parse_gps_position_reading(raw))
        if (reading['speedError'] is None and
                reading['positionError'] is None):
            break
    return None, reading


def parse_gps_speed_reading(line_of_output):
    """
    It parses the line of the output from the GPS reader that contains the
    speed.
    """
    as_string = line_of_output.decode('utf-8')
    parsed = GPS_SPEED_PARSER.parse(as_string)
    if parsed is None:
        return None, "Could not parse GPS speed reading."
    # The raw reading is in km/hour, so to convert it into metres / second
    # it is multiplied by 1000 / (60 x 60) = 0.277777778.
    return parsed[-2] * 0.277777778, None


GPS_SPEED_PARSER = parse.compile('$GPVTG,{},N,{:f},K{}')


def parse_gps_position_reading(line_of_output):
    """
    It parses a line of the output from the GPS reader that contains the
    position.
    """
    as_string = line_of_output.decode('utf-8')
    parsed = GPS_POSITION_PARSER.parse(as_string)
    if parsed is None:
        return None, "Could not parse GPS position reading."
    if parsed[1] == 'N':
        latitude_sign = 1
    if parsed[1] == 'S':
        latitude_sign = -1
    if parsed[3] == 'E':
        longitude_sign = 1
    if parsed[3] == 'W':
        longitude_sign = -1
    return (
        {'latitude': latitude_sign * angle_minutes_to_float(str(parsed[0])),
         'longitude': longitude_sign * angle_minutes_to_float(str(parsed[2]))},
        None)


def angle_minutes_to_float(angle):
    """
    It converts the angle from degrees and minutes to just degrees.

    The input angle is a string like "5016.81116". Everything after
    the decimal point are the decimals of the minutes of the angle,
    the two digits immediately to the left of the decimal point are
    the whole minutes, and everything to the left of that is the
    whole degrees.  One degree is 60 minutes.
    """
    first_half, minutes_decimals = angle.split('.')
    whole_minutes = first_half[-2:]
    degrees = first_half[:-2]
    print "angle<" + angle + ">"
    print "whole_minutes<" + str(whole_minutes) + ">"
    print "degrees<" + str(degrees) + ">"
    print "minutes_decimals<" + minutes_decimals + ">"
    if len(degrees) == 0:
        floatdegrees = 0
    else:
        floatdegrees = float(degrees)
    return floatdegrees + float(whole_minutes + '.' + minutes_decimals) / 60


GPS_POSITION_PARSER = parse.compile('$GPGLL,{:f},{:w},{:f},{:w},{:S}\r\n')


def test_GPS():
    with serial.Serial('/dev/ttyACM0', baudrate=115200) as gps_port:
        while True:
            time.sleep(1)
            print readGps(gps_port)


def testRoutePlanner():
    start = {
        'latitude': 52.237800,
        'longitude': 0.155456}

    end = {
        'latitude': 52.22767,
        'longitude': 0.149673}

    print plan_route.main(start, end)


def makeHandles():
    movehandle, err = setupMoveSensor()
    if err is not None:
        return None, err

    cam0, err = connectToCam(0)
    if err is not None:
        return None, err

    cam1, err = connectToCam(1)
    if err is not None:
        return None, err

    cam2, err = connectToCam(2)
    if err is not None:
        return None, err

    return {
        'cam0': cam0,
        'cam1': cam1,
        'cam2': cam2,
        'moveSensor': movehandle,
        'gps': serial.Serial('/dev/ttyACM0', baudrate=115200)
        }, None


def readSensors(handles):
    return {
        'gps': readGps(handles['gps']),
        'motion': readMoveSensor(handles['moveSensor']),
        'cam0': takePhoto(handles['cam0']),
        'cam1': takePhoto(handles['cam1']),
        'cam2': takePhoto(handles['cam2'])}


DATADIR = "/home/pi/data/" + str(time.time())


def writeDataToFile(data):
    dataDir = DATADIR + '/' + str(time.time())
    os.mkdir(dataDir)
    pic0, err0 = data['cam0']
    pic1, err1 = data['cam1']
    pic2, err2 = data['cam2']
    with open(dataDir + '/data.json', 'w') as dataFile:
        json.dump(
            {'gps': data['gps'],
             'motion': data['motion'],
             'cam0err': err0,
             'cam1err': err1,
             'cam2err': err2},
            dataFile)
    cv2.imwrite(dataDir + '/im0.jpg', pic0)
    cv2.imwrite(dataDir + '/im1.jpg', pic1)
    cv2.imwrite(dataDir + '/im2.jpg', pic2)


DESTINATION = {
    'latitude': 29,
    'longitude': 42}


HOME = {
    'latitude': 33,
    'longitude': 44}


def initState():
    return {
        'destination': DESTINATION,
        'location': {
            'latitude': HOME['latitude'],
            'longitude': HOME['longitude']},
        'outwardBound': True}


def main():
    os.mkdir(DATADIR)
    window = tk.Tk()
    canvas = tk.Canvas(window, width=400, height=400)
    canvas.pack()

    handles, err = makeHandles()
    if err is not None:
        print err
        return

    state = initState()

    while True:
        print "top of loop"
        sensorReadings = readSensors(handles)

        gpsReadings, _ = sensorReadings['gps']
        if gpsReadings is not None:
            state['location'] = gpsReadings['position']
        writeDataToFile(sensorReadings)

        if plan_route.distance_between(state['location'], DESTINATION) < 30:
            state['outwardBound'] = False
            state['destination'] = HOME

        if (not state['outwardBound'] and
                plan_route.distancebetween(state['location'], HOME) < 30):
            print "Arrived home. Exiting."
            return

        desiredDirection, err = plan_route.main(
            state['location'],
            state['destination'])
        if err is not None:
            print err
            continue
        headingRadians = sensorReadings['motion']['heading'] * math.pi / 180
        correctionAngle = subtractAngles(desiredDirection, headingRadians)

        canvas.create_line(
            200,
            200,
            200*math.cos(correctionAngle),
            200*math.sin(correctionAngle),
            arrow=tk.LAST,
            width=10,
            arrowshape=(30, 40, 10))
        canvas.delete("all")
        time.sleep(0.05)

    window.mainloop()


main()
