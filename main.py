# Tested in Python 2.7.13.

from Adafruit_BNO055 import BNO055 as move_sensor
import time
import cv2

def ioCalibrateMoveSensor(bno):
    cal = bno.get_calibration_status()
    while not isCalibrated(cal):
        time.sleep(0.5)
        print calibrationMsg(cal)
        cal = bno.get_calibration_status()

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

def isCalibrated(calDat):
    sys, gyro, accel, mag = calDat
    return sys == 3 and gyro == 3 and accel == 3 and mag == 3
        
def ioConnectToMoveSensor():
    handle = move_sensor.BNO055(serial_port='/dev/serial0', rst=18)
    if not handle.begin():
        return (None, "Could not connect to movement sensor.")
    return (None, handle)

def ioReadMoveSensor(bno):
    heading, roll, pitch = bno.read_euler() 
    x, y, z = bno.read_accelerometer()
    return {
        'heading': heading,
        'roll': roll,
        'pitch': pitch,
        'accelx': x,
        'accely': y,
        'accelz': z}

def ioConnectToCam(camNum):
    handle = cv2.VideoCapture(camNum)
    if not handle.isOpened():
        return None, "Could not connect to webcam {}".format(camNum)
    return handle, None

def ioTakePhoto(camHandle):
    ok, photo = camHandle.read()
    if not ok:
        return None, "Could not take photo."
    return photo, None

def toBlackAndWhite(im):
    return cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)

def readGps():
    with serial.Serial('/dev/ttyACM0', baurate=115200) as gps_port:
        return gps_port


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
            reading['speedError'], reading['speed'] = (
                parse_gps_speed_reading(raw))
        if reading['positionError'] is not None:
            reading['positionError'], reading['position'] = (
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
    return (plan_route.MapPosition(
        latitude=latitude_sign * angle_minutes_to_float(str(parsed[0])),
        longitude=longitude_sign * angle_minutes_to_float(str(parsed[2]))), None)


GPS_POSITION_PARSER = parse.compile('$GPGLL,{:f},{:w},{:f},{:w},{:S}\r\n')


with serialSerial('/dev/ttyACM0', baudrate=115200) as gps_port:
    print readGps(gps_port)
