#!/usr/bin/env python2
# coding=utf-8
import sys, json, time
import serial
import httplib, urllib, urlparse
import threading
import base64, hashlib
import functools
import numpy as np
import cv2

from dronekit import connect, VehicleMode
from picamera.array import PiRGBArray
from picamera import PiCamera
from PyQt4.QtCore import *
from PyQt4.QtGui import *

# vars ->
lock   = threading.Lock()
video1 = QImage()
video2 = QImage()

global mode
global rec
global zoom
global display
global wifi
global armed

global conn
# global ser
global vehicle

global zoom_value
global alt_value
global sat_value
global gs_value
global yaw_value
global bat_cur
global bat_volt
global bat_level
global zenlogo

global roll
global nick
global yaw
global alt
global sat
global land

global appWidth
global appHeight
global imgHeight

global batNumer
global batDenom
global batLevel1
global batLevel2

output     = "HDMI" #HDMI/RGB

mode       = "still"
rec        = "off"
zoom       = 0
display    = 0
armed      = 0
wifi       = True

sat_value  = "0 SAT. No Fix"
alt_value  = "0 m"
gs_value   = "0 km/h"
yaw_value  = "0"
zoom_value = "0X"
bat_volt   = "0 V"
bat_cur    = "0 A"
bat_level  = "0 %"

roll = 0
nick = 0
yaw  = 0
alt  = 0
sat  = 0
land = 0

batNumer  = 0
batDenom  = 4
batLevel1 = 0
batLevel2 = 0

#HDMI
if output == "HDMI":
    appWidth  = 800
    appHeight = 480
    imgHeight = 310

#RGB
elif output == "RGB":
    appWidth  = 942
    appHeight = 560
    imgHeight = 340

print ">>> OUTPUT: " + output

# images ->
zenlogo        = "/home/pi/Documents/QX30/img/zen-drones.png"
imgMode1       = "/home/pi/Documents/QX30/img/imgMode1.png"
imgMode2       = "/home/pi/Documents/QX30/img/imgMode2.png"
imgMode3       = "/home/pi/Documents/QX30/img/imgMode3.png"
imgSatOn       = "/home/pi/Documents/QX30/img/imgSatOn.png"
imgSatOff      = "/home/pi/Documents/QX30/img/imgSatOff.png"
imgWifiOn      = "/home/pi/Documents/QX30/img/imgWifiOn.png"
imgWifiOff     = "/home/pi/Documents/QX30/img/imgWifiOff.png"
imgBat000      = "/home/pi/Documents/QX30/img/batt_000.png"
imgBat020      = "/home/pi/Documents/QX30/img/batt_020.png"
imgBat040      = "/home/pi/Documents/QX30/img/batt_040.png"
imgBat060      = "/home/pi/Documents/QX30/img/batt_060.png"
imgBat080      = "/home/pi/Documents/QX30/img/batt_080.png"
imgBat100      = "/home/pi/Documents/QX30/img/batt_100.png"
imgBat999      = "/home/pi/Documents/QX30/img/batt_999.png"
imgSD          = "/home/pi/Documents/QX30/img/sdcard.png"
imgStatus      = "/home/pi/Documents/QX30/img/status.png"
imgZoom        = "/home/pi/Documents/QX30/img/zoom.png"
imgAlt         = "/home/pi/Documents/QX30/img/alt.png"
imgYaw         = "/home/pi/Documents/QX30/img/yaw.png"
imgSpeed       = "/home/pi/Documents/QX30/img/speed.png"
imgCurrent     = "/home/pi/Documents/QX30/img/current.png"

hud            = "/home/pi/Documents/QX30/hud/hud.png"
hudPitch       = "/home/pi/Documents/QX30/hud/pitch.png"
hudRoll        = "/home/pi/Documents/QX30/hud/roll.png"
hudCenter      = "/home/pi/Documents/QX30/hud/center.png"
hudAlt         = "/home/pi/Documents/QX30/hud/alt.png"
hudClimb       = "/home/pi/Documents/QX30/hud/climb.png"


fontSmall   = "color:#c2b59b; font-family:Dosis; font-weight:bold; font-size:12px;"
fontMedium1 = "color:#c2b59b; font-family:Dosis; font-weight:normal; font-size:20px; text-transform:uppercase"
fontMedium2 = "color:#ff6f49; font-family:Dosis; font-weight:bold; font-size:20px; text-transform:uppercase"
fontBig1    = "color:#c2b59b; font-family:Dosis; font-weight:normal; font-size:26px;"
fontBig2    = "color:#ff6f49; font-family:Dosis; font-weight:normal; font-size:26px;"


# init ->
# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (560, 320)
camera.framerate = 30
rawCapture = PiRGBArray(camera, size=(560, 320))
# allow the camera to warmup
time.sleep(1)

# initialize wifi and seral connections
conn    = httplib.HTTPConnection("192.168.122.1", 8080)
#ser     = serial.Serial('/dev/ttyUSB0', 57600)
vehicle = connect('/dev/ttyAMA0', wait_ready=True, baud=57600, heartbeat_timeout=60)

# initialize vars for sony com
pId = 0
headers = {"Content-type": "text/plain", "Accept": "*/*", "X-Requested-With": "com.sony.playmemories.mobile"}
AUTH_CONST_STRING = "90adc8515a40558968fe8318b5b023fdd48d3828a2dda8905f3b93a3cd8e58dc"
METHODS_TO_ENABLE = "camera/setFlashMode:camera/getFlashMode:camera/getSupportedFlashMode:camera/getAvailableFlashMode:camera/setExposureCompensation:camera/getExposureCompensation:camera/getSupportedExposureCompensation:camera/getAvailableExposureCompensation:camera/setSteadyMode:camera/getSteadyMode:camera/getSupportedSteadyMode:camera/getAvailableSteadyMode:camera/setViewAngle:camera/getViewAngle:camera/getSupportedViewAngle:camera/getAvailableViewAngle:camera/setMovieQuality:camera/getMovieQuality:camera/getSupportedMovieQuality:camera/getAvailableMovieQuality:camera/setFocusMode:camera/getFocusMode:camera/getSupportedFocusMode:camera/getAvailableFocusMode:camera/setStillSize:camera/getStillSize:camera/getSupportedStillSize:camera/getAvailableStillSize:camera/setBeepMode:camera/getBeepMode:camera/getSupportedBeepMode:camera/getAvailableBeepMode:camera/setCameraFunction:camera/getCameraFunction:camera/getSupportedCameraFunction:camera/getAvailableCameraFunction:camera/setLiveviewSize:camera/getLiveviewSize:camera/getSupportedLiveviewSize:camera/getAvailableLiveviewSize:camera/setTouchAFPosition:camera/getTouchAFPosition:camera/cancelTouchAFPosition:camera/setFNumber:camera/getFNumber:camera/getSupportedFNumber:camera/getAvailableFNumber:camera/setShutterSpeed:camera/getShutterSpeed:camera/getSupportedShutterSpeed:camera/getAvailableShutterSpeed:camera/setIsoSpeedRate:camera/getIsoSpeedRate:camera/getSupportedIsoSpeedRate:camera/getAvailableIsoSpeedRate:camera/setExposureMode:camera/getExposureMode:camera/getSupportedExposureMode:camera/getAvailableExposureMode:camera/setWhiteBalance:camera/getWhiteBalance:camera/getSupportedWhiteBalance:camera/getAvailableWhiteBalance:camera/setProgramShift:camera/getSupportedProgramShift:camera/getStorageInformation:camera/startLiveviewWithSize:camera/startIntervalStillRec:camera/stopIntervalStillRec:camera/actFormatStorage:system/setCurrentTime"

# initialize openCV Object tracker
face_cascade = cv2.CascadeClassifier('/home/pi/opencv-3.1.0/data/haarcascades/haarcascade_frontalface_default.xml')
body_cascade = cv2.CascadeClassifier('/home/pi/opencv-3.1.0/data/haarcascades/haarcascade_fullbody.xml')
color = (73, 111, 255)
flags = cv2.CASCADE_SCALE_IMAGE
hog   = cv2.HOGDescriptor()
hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())


# dynamic labels ->
class SatLabel(QLabel):
    def __init__(self):
        QLabel.__init__(self)

    def paintEvent(self, event):
        global sat
        if sat < 6:
            imgSat = imgSatOff
        elif sat >= 6:
            imgSat = imgSatOn

        img = QImage(imgSat)
        self.setPixmap(QPixmap.fromImage(img))
        QLabel.paintEvent(self,event)

class WifiLabel(QLabel):
    def __init__(self):
        QLabel.__init__(self)

    def paintEvent(self, event):
        global wifi
        if wifi == True:
            imgWifi = imgWifiOn
        elif wifi == False:
            imgWifi = imgWifiOff

        img = QImage(imgWifi)
        self.setPixmap(QPixmap.fromImage(img))
        QLabel.paintEvent(self,event)
        
class Bat1Label(QLabel):
    def __init__(self):
        QLabel.__init__(self)

    def paintEvent(self, event):
        global batLevel1
        if batLevel1 <= 15:
            imgBat = imgBat000
        elif batLevel1 > 15 and batLevel1 <= 30:
            imgBat = imgBat020
        elif batLevel1 > 30 and batLevel1 <= 45:
            imgBat = imgBat040
        elif batLevel1 > 45 and batLevel1 <= 65:
            imgBat = imgBat060
        elif batLevel1 > 65 and batLevel1 <= 85:
            imgBat = imgBat080
        elif batLevel1 > 85:
            imgBat = imgBat100
            
        img = QImage(imgBat)
        self.setPixmap(QPixmap.fromImage(img))
        QLabel.paintEvent(self,event)

class Bat2Label(QLabel):
    def __init__(self):
        QLabel.__init__(self)

    def paintEvent(self, event):
        global batLevel2
        if batLevel2 <= 0.25:
            imgBat = imgBat000
        elif batLevel2 > 0.25 and batLevel2 <= 0.5:
            imgBat = imgBat040
        elif batLevel2 > 0.5 and batLevel2 <= 0.75:
            imgBat = imgBat080
        elif batLevel2 > 0.75 and batLevel2 <= 1:
            imgBat = imgBat100
        elif batLevel2 > 1:
            imgBat = imgBat999

        img = QImage(imgBat)
        self.setPixmap(QPixmap.fromImage(img))
        QLabel.paintEvent(self,event)

class ModeLabel(QLabel):
    def __init__(self):
        QLabel.__init__(self)

    def paintEvent(self, event):
        global mode
        if mode == "still":
            imgMode = imgMode1
        elif mode == "movie":
            imgMode = imgMode2
        if rec == "on":
            imgMode = imgMode3

        img = QImage(imgMode)
        self.setPixmap(QPixmap.fromImage(img))
        QLabel.paintEvent(self,event)



class HudDisplay(QLabel):
    def __init__(self):
        QLabel.__init__(self) 
        self.canvas_w = appWidth/2
        self.canvas_h = imgHeight
        self.setScaledContents(True)
        
    def paintEvent(self, event):
        global lock

        lock.acquire()
        
        img = QImage(hud)
        
        try:
            rollimg  = QImage(hudRoll)
            pitchimg = QImage(hudPitch)
            center   = QImage(hudCenter)
            altlevel = QImage(hudAlt)
                     
            qp = QPainter()
            qp.begin(rollimg)
            target0 = QRectF(0, 50*nick, 320, 320)
            qp.drawImage(target0, pitchimg)
            qp.end()
            
            rollimg  = rollimg.transformed(QTransform().rotate(roll*56.25))
            rollpix  = QPixmap(rollimg)
            
            qp.begin(img)
            target1 = QRectF((400-rollpix.width())/2, (400-rollpix.height())/2-40, rollpix.width(), rollpix.height())
            target2 = QRectF(0, 250-alt*2, 30, 20)
            target3 = QRectF(140, 150, 120, 20)
            qp.drawImage(target1, rollimg)
            qp.drawImage(target2, altlevel)
            qp.drawImage(target3, center)
            qp.end()
            
            self.setPixmap(QPixmap.fromImage(img))
        finally:
            lock.release()
        QLabel.paintEvent(self,event)

# live views ->
class SonyDisplay(QLabel):
    def __init__(self):
        QLabel.__init__(self)
        if display == 0:
            self.canvas_w = appWidth/2
            self.canvas_h = imgHeight
        elif display == 2:
            self.canvas_w = appWidth
            self.canvas_h = appHeight
        self.setScaledContents(True)
        
    def paintEvent(self, event):
        global video1
        global lock
        lock.acquire()
        try:
            self.setPixmap(QPixmap.fromImage(video1))
        finally:
            lock.release()
        QLabel.paintEvent(self, event)

class PicamDisplay(QLabel):
    def __init__(self):
        QLabel.__init__(self) 
        if display == 0:
            self.canvas_w = appWidth/2
            self.canvas_h = imgHeight
        elif display == 2:
            self.canvas_w = appWidth
            self.canvas_h = appHeight
        self.setScaledContents(True)
        
    def paintEvent(self, event):
        global video2
        global lock

        lock.acquire()        
        try:            
            self.setPixmap(QPixmap.fromImage(video2))
        finally:
            lock.release()
        QLabel.paintEvent(self,event)
        
# helper functions ->
def readLineRC(port):
    while True:
        ch = port.readline()
        return ch

def postRequest(conn, target, req):
    global pId
    pId += 1
    req["id"] = pId
    conn.request("POST", "/sony/" + target, json.dumps(req), headers)
    response = conn.getresponse()
    data = json.loads(response.read().decode("UTF-8"))
    if data["id"] != pId:
        return {}
    return data

def exitWithError(conn, message):
    conn.close()
    sys.exit(1)

def parseUrl(url):
    parsedUrl = urlparse.urlparse(url)
    return parsedUrl.hostname, parsedUrl.port, parsedUrl.path + "?" + parsedUrl.query, parsedUrl.path[1:]
     
def drawDetections(img, rects,shrink,string,thickness = 1):
    i = 0
    for x, y, w, h, in rects:
        i+=1
        if shrink == True:
            pad_w, pad_h = int(0.15*w), int(0.05*h)
        else:
            pad_w, pad_h = w, h

        #cv2.rectangle(img, (x, y-15), (x+90, y), color, -1)
        #cv2.putText(img, string+" "+str(i), (x+3, y-5), cv2.FONT_HERSHEY_PLAIN, 0.5, (255, 255, 255))
        cv2.rectangle(img, (x, y-3), (x+20, y), color, -1)
        cv2.rectangle(img, (x-3, y-3), (x, y+20), color, -1)
        cv2.rectangle(img, (x+pad_w, y+pad_h), (x+w-pad_w, y+h-pad_h), color, thickness)

def connected(host="192.168.122.1", port=8080, timeout=3):
    global wifi
    tmp = wifi
    try:
        socket.setdefaulttimeout(timeout)
        socket.socket(socket.AF_INET, socket.SOCK_STREAM).connect((host, port))
        wifi = True
        if tmp == False and wifi == True:
            print (">>> WIFI CONNECTED")
            communication = threading.Thread(target = communicationThread)
            communication.start()
            initValues()
            setMemory()
    except Exception as ex:
        #communication.stop()
        if tmp == True:
            wifi = False
            print (">>> " + ex.message)
        pass
    
# threads ->
def communicationThread():

    resp = postRequest(conn, "camera", {"method": "startRecMode", "params": [], "version": "1.0"})
    resp = postRequest(conn, "camera", {"method": "getVersions", "params": []})
    if resp["result"][0][0] != "1.0":
        exitWithError(conn, "Unsupported version")

    resp = postRequest(conn, "accessControl", {"method": "actEnableMethods", "params": [{"methods": "", "developerName": "", "developerID": "", "sg": ""}], "version": "1.0"})
    dg = resp["result"][0]["dg"]

    h = hashlib.sha256()
    h.update(bytes(AUTH_CONST_STRING + dg))
    sg = base64.b64encode(h.digest()).decode("UTF-8")

    resp = postRequest(conn, "camera", {"method": "startLiveview", "params": [], "id": 1, "version": "1.0"})
    liveview = threading.Thread(target = sonyliveviewThread, args = (resp["result"][0],))
    liveview.start()
    
def sonyliveviewThread(url):
    global video1
    global lock
    
    host, port, address, img_name = parseUrl(url)
    connLive = httplib.HTTPConnection(host, port)
    connLive.request("GET", address)
    response = connLive.getresponse()

    if response.status == 200:
        buf = b''
        c = 0
        while response.status==200:
            nextPart = response.read(1024)

            # TODO: It would be better to use description from the documentation (page 51) for parsing liveview stream
            jpegStart = nextPart.find(b'\xFF\xD8\xFF')
            jpegEnd = nextPart.find(b'\xFF\xD9')
            if jpegEnd != -1:
                c += 1
                buf += nextPart[:jpegEnd + 2]
                
                lock.acquire()
                try:
                    if display == 3: #1 for detecting faces
                        cvbuf = np.frombuffer(buf, dtype='uint8')
                        img = cv2.imdecode(cvbuf, flags=1)
                        foundFaces=face_cascade.detectMultiScale(img, scaleFactor=1.3, minNeighbors=5, minSize=(50, 50), flags=flags)
                        drawDetections(img,foundFaces,False,"DETECTED: FACE")
                        #foundPeople,w=hog.detectMultiScale(img, winStride=(8,8), padding=(32,32), scale=1.05)
                        #drawDetections(img,foundPeople,True,"DETECTED: PERSON")
                        r, newImg = cv2.imencode(".jpg",img)
                        video1.loadFromData(newImg)
                        webimage = buf
                    else:
                        video1.loadFromData(buf)
                        #webimage = buf
                    
                finally:
                    lock.release()

            if jpegStart != -1:
                buf = nextPart[jpegStart:]
            else:
                buf += nextPart

def picamliveviewThread():
    global video2
    global lock
    
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        lock.acquire()
        try:

            if display == 2:
                img = frame.array
                foundFaces=face_cascade.detectMultiScale(img, scaleFactor=1.3, minNeighbors=5, minSize=(50, 50), flags=flags)
                drawDetections(img,foundFaces,False,"DETECTED: FACE")
                #foundPeople,w=hog.detectMultiScale(img, winStride=(8,8), padding=(32,32), scale=1.05)
                #drawDetections(img,foundPeople,True,"DETECTED: PERSON")

                newimg = img.copy(order='C')
                height, width, byteValue = newimg.shape
                byteValue = byteValue * width
                cv2.cvtColor(newimg, cv2.COLOR_BGR2RGB, newimg)
                video2 = QImage(newimg, width, height, byteValue, QImage.Format_RGB888)
                rawCapture.truncate(0)


            else:
                img = frame.array.copy(order='C')
                height, width, byteValue = img.shape
                byteValue = byteValue * width
                cv2.cvtColor(img, cv2.COLOR_BGR2RGB, img)
                #r, webimage = cv2.imencode(".jpg",img)
                video2 = QImage(img, width, height, byteValue, QImage.Format_RGB888)
                rawCapture.truncate(0)
            
        finally:
            lock.release()

# this part and the Arduiono can be removed as we are able to read channels via mavlink        
def arduinoSerialThread():
    global display
    rcvTmp =""
    while 1:
        rcv = readLineRC(ser)
        rcv = (str(rcv)).replace("\r\n","")
        print (">>> MESSAGE FROM ARDUINO: " + rcv)
        if rcv == "SHOOT":
            takePic()
            rcvTmp = rcv
        if rcv == "STILL" and rcvTmp != rcv:
            setModeStill()
            rcvTmp = rcv
        if rcv == "MOVIE" and rcvTmp != rcv:
            setModeMovie()
            rcvTmp = rcv
        if rcv == "ZOOMIN" and rcvTmp != rcv:
            zoomIn()
            rcvTmp = rcv
        if rcv == "ZOOMOUT" and rcvTmp != rcv:
            zoomOut()
            rcvTmp = rcv
        if rcv == "ZOOMOFF" and rcvTmp != rcv:
            zoomInStop()
            zoomOutStop()
            rcvTmp = rcv
        if rcv == "SHOWALL" and rcvTmp != rcv:
            display = 0
            box.reinit()
            rcvTmp = rcv
            print(">>> CALL CHANGE LAYOUT WITH MODE: " + str(display))
        if rcv == "SHOWSONY" and rcvTmp != rcv:
            display = 1
            box.reinit()
            rcvTmp = rcv
            print(">>> CALL CHANGE LAYOUT WITH MODE: " + str(display))
        if rcv == "SHOWPILOT" and rcvTmp != rcv:
            display = 2
            box.reinit()
            rcvTmp = rcv
            print(">>> CALL CHANGE LAYOUT WITH MODE: " + str(display))

def sonyGetBatLevelThread():

    def getBatLevel():
        global wifi
        global batLevel2
        if wifi == True:
            try:
                resp = postRequest(conn, "camera", {"method": "getEvent", "params": [False], "version": "1.2"})
                bat  = (resp["result"][56]["batteryInfo"][0])
                batNumer = bat["levelNumer"]
                batDenom = bat["levelDenom"]
                batStatus =bat["additionalStatus"]
                if batStatus == "charging":
                    batLevel2 = 2
                else:
                    batLevel2 = float(batNumer)/float(batDenom)
            except Exception as ex:
                print (">>> " + ex.message)
        else:
            connected()
    while 1:
        getBatLevel()
        time.sleep(15)
        

def testingThread(mode = "cam"):
    global display
    def changeMode():
        print(">>> CALL CHANGE LAYOUT WITH MODE: " + str(display))
        box.reinit()

    if mode == "cam":
        time.sleep(5)
        zoomIn()

        time.sleep(5)
        zoomInStop()

        time.sleep(5)
        zoomOut()

        time.sleep(6)
        zoomOutStop()

    if mode == "display":
        time.sleep(5)
        display = 1
        changeMode()

        time.sleep(5)
        display = 2
        changeMode()

        time.sleep(5)
        display = 0
        changeMode()
    
    
# callbacks ->
def sat_callback(self, attr_name, value):
    global sat_value
    global sat
    sat = value.satellites_visible
    fix = value.fix_type

    if fix < 2:
        msg = "NO FIX"
    elif fix == 2:
        msg = " 2D"
    elif fix == 3:
        msg = " 3D"
    elif fix > 3:
        msg = " 4D"
    sat_value = str(sat) + " SAT " + msg
    box.satcountDisplay.setText(sat_value)

def alt_callback(self, attr_name, value):
    global alt_value
    global alt
    alt = value.alt
    alt_value = str(round(alt,1)) + " m"
    box.altDisplay.setText(alt_value)

def status_callback(self, attr_name, value):
    global armed
    armed = value.armed

# the idea was to detect landing and switch on some leds    
def land_callback(self, attr_name, value):
    global land
    altrel = value.alt
    if(altrel < 3 and land == 0 and armed == 1):
        land = 1
        # ser.write("1")
    elif(altrel > 3 and land == 1 and armed == 1):
        land = 0
        # ser.write("0")
    else:
        land = 0
        # ser.write("0")

def gs_callback(self, attr_name, value):
    global gs_value
    gs_value = str(round(value,1)) + " km/h"
    box.speedDisplay.setText(gs_value)
      
def bat_callback(self, attr_name, value):
    global bat_cur
    global bat_volt
    global bat_level
    global bat_Level1
    if value.current:
        bat_cur = str(round(value.current,1)) + " A"
    else:
        bat_cur = "0.0 A"
    if value.level:
        bat_level = str(round(value.level,1)) + " %"
        bat_icon = value.level
    else:
        bat_level = "0 %"
        bat_icon = 0
    bat_volt = bat_level + "  " + str(round(value.voltage,1)) + " V"
    box.batDisplay.setText(bat_volt)
    box.curDisplay.setText(bat_cur)

def att_callback(self, attr_name, value):
    global roll
    global nick
    global yaw
    global yaw_value
    roll = round(value.roll,1)
    nick = round(value.pitch,1)
    yaw = round(value.yaw,1)
    yaw_value  = str(round(yaw*56.25,1)) + QString.fromUtf8("° ")
    box.yawDisplay.setText(yaw_value)
    
# main GUI ->
class Box(QDialog):

    def __init__(self, parent=None, margin=-10, spacing=-10):
        super(Box, self).__init__(parent)
        self.initUI()

        self.setContentsMargins(-11,-11,-11,-12)
        #self.setSpacing(spacing)


    def initUI(self):
        global mode
        global rec
        global wifi

        if display == 0:
            self.cam1Label = QLabel("  OPERATOR CAM")
            self.cam2Label = QLabel("  PILOT CAM")

        if display == 1:
            self.cam1Label = QLabel("  OPERATOR CAM")
            self.cam2Label = QLabel("")

        if display == 2:
            self.cam1Label = QLabel("  PILOT CAM")
            self.cam2Label = QLabel("")

        self.cam1Label.setStyleSheet(fontMedium2)
        self.cam2Label.setStyleSheet(fontMedium2)

        self.zoomLabel = QLabel()
        self.zoomLabel.setPixmap(QPixmap(imgZoom))

        self.statusLabel = QLabel()
        self.statusLabel.setPixmap(QPixmap(imgStatus))

        self.memoryLabel = QLabel()
        self.memoryLabel.setPixmap(QPixmap(imgSD))

        self.zoomDisplay  = QLabel("0 X")
        self.zoomDisplay.setStyleSheet(fontBig2)

        self.statusDisplay  = QLabel("IDLE")
        self.statusDisplay.setStyleSheet(fontBig2)

        self.memoryDisplay = QLabel("MEMORY")
        self.memoryDisplay.setStyleSheet(fontBig2)

        self.batDisplay = QLabel("0%  0.0 V ")
        self.batDisplay.setStyleSheet(fontBig2)

        self.curLabel = QLabel()
        self.curLabel.setPixmap(QPixmap(imgCurrent))

        self.curDisplay = QLabel("0.0 A ")
        self.curDisplay.setStyleSheet(fontBig2)

        self.satcountDisplay = QLabel("0 SATs")
        self.satcountDisplay.setStyleSheet(fontBig2)

        self.altLabel = QLabel()
        self.altLabel.setPixmap(QPixmap(imgAlt))

        self.altDisplay = QLabel("0.0 m ")
        self.altDisplay.setStyleSheet(fontBig2)

        self.distLabel = QLabel("DIST ")
        self.distLabel.setStyleSheet(fontSmall)
        self.distLabel.setAlignment(Qt.AlignRight|Qt.AlignVCenter)

        self.distDisplay = QLabel("0 m ")
        self.distDisplay.setStyleSheet(fontBig2)

        self.speedLabel = QLabel()
        self.speedLabel.setPixmap(QPixmap(imgSpeed))

        self.speedDisplay = QLabel("0.0 km/h ")
        self.speedDisplay.setStyleSheet(fontBig2)

        self.yawLabel = QLabel()
        self.yawLabel.setPixmap(QPixmap(imgYaw))

        self.yawDisplay = QLabel(QString.fromUtf8("0° "))
        self.yawDisplay.setStyleSheet(fontBig2)

        self.zen = QLabel()
        self.zen.setPixmap(QPixmap(zenlogo))
        self.zen.setAlignment(Qt.AlignCenter|Qt.AlignVCenter)

        self.imgDisplay  = SonyDisplay()
        self.picDisplay  = PicamDisplay()
        self.hudDisplay  = HudDisplay()

        self.imgBigDisplay = SonyDisplay()
        self.picBigDisplay = PicamDisplay()

        self.picBigDisplay.setMaximumSize(appWidth, appHeight)
        self.imgBigDisplay.setMaximumSize(appWidth, appHeight)

        self.imgDisplay.setMaximumSize(appWidth/2, imgHeight)
        self.picDisplay.setMaximumSize(appWidth/2, imgHeight)
        self.hudDisplay.setMaximumSize(appWidth/2, imgHeight)
        self.hudDisplay.setAlignment(Qt.AlignCenter|Qt.AlignVCenter)

        satDisplay  = SatLabel()
        bat1Display = Bat1Label()
        bat2Display = Bat2Label()
        wifiDisplay = WifiLabel()
        modeDisplay = ModeLabel()

        self.main = QGridLayout()
        self.main.setSpacing(0)

        self.main.addWidget(self.imgBigDisplay,0,0)
        self.main.addWidget(self.picBigDisplay,0,0)

        self.grid = QGridLayout()
        self.grid.setSpacing(0)

        self.grid.addWidget(self.cam1Label,0,0)
        self.grid.addWidget(self.cam2Label,0,1)
        self.grid.addWidget(self.imgDisplay,1,0)
        self.grid.addWidget(self.picDisplay,1,1)
        self.grid.addWidget(self.hudDisplay,1,1)

        self.grid.setRowMinimumHeight(1,imgHeight)
        self.grid.setColumnMinimumWidth(0,appWidth/2)
        self.grid.setColumnMinimumWidth(1,appWidth/2)

        self.osd1 = QGridLayout()
        self.osd1.addWidget(bat2Display,0,0)
        self.osd1.addWidget(self.zoomLabel,0,1)
        self.osd1.addWidget(self.zoomDisplay,0,2)

        self.osd1.addWidget(modeDisplay,1,0)
        self.osd1.addWidget(self.statusLabel,1,1)
        self.osd1.addWidget(self.statusDisplay,1,2)

        self.osd1.addWidget(wifiDisplay,2,0)
        self.osd1.addWidget(self.memoryLabel,2,1)
        self.osd1.addWidget(self.memoryDisplay,2,2)

        self.osd1.setColumnMinimumWidth(0,55)
        self.osd1.setColumnMinimumWidth(1,55)
        self.osd1.setColumnMinimumWidth(2,290)
        self.osd1.setRowMinimumHeight(0,50)
        self.osd1.setRowMinimumHeight(1,50)
        self.osd1.setRowMinimumHeight(2,50)

        self.osd2 = QGridLayout()
        self.osd2.addWidget(bat1Display,0,0)
        self.osd2.addWidget(self.batDisplay,0,1)
        self.osd2.addWidget(self.curLabel,0,2)
        self.osd2.addWidget(self.curDisplay,0,3)
        self.osd2.addWidget(satDisplay,1,0)
        self.osd2.addWidget(self.satcountDisplay,1,1)
        self.osd2.addWidget(self.altLabel,1,2)
        self.osd2.addWidget(self.altDisplay,1,3)
        self.osd2.addWidget(self.speedLabel,2,0)
        self.osd2.addWidget(self.speedDisplay,2,1)
        self.osd2.addWidget(self.yawLabel,2,2)
        self.osd2.addWidget(self.yawDisplay,2,3)

        self.osd2.setColumnMinimumWidth(0,55)
        self.osd2.setColumnMinimumWidth(1,180)
        self.osd2.setColumnMinimumWidth(2,55)
        self.osd2.setColumnMinimumWidth(3,110)
        self.osd2.setRowMinimumHeight(0,50)
        self.osd2.setRowMinimumHeight(1,50)
        self.osd2.setRowMinimumHeight(2,50)

        self.grid.addLayout(self.osd1,2,0)
        self.grid.addLayout(self.osd2,2,1)

        self.main.addLayout(self.grid,0,0)

        self.setLayout(self.main)
        self.initvals()
        self.reinit()

    def initvals(self):
        tmpMode = postRequest(conn, "camera", {"method": "getShootMode", "params": [], "id": 1, "version": "1.0"})
        mode = str(tmpMode["result"][0])
        tmprec = postRequest(conn, "camera", {"method": "getEvent", "params": [False], "id": 4, "version": "1.0"})
        self.statusDisplay.setText(str(tmprec["result"][1]["cameraStatus"]))

        feedback = postRequest(conn, "camera", {"method": "getEvent", "params": [False], "id": 4, "version": "1.0"})
        zoom = feedback["result"][2]["zoomPosition"]
        zoom_value = str(round(zoom*0.333,1)) + " X"
        self.zoomDisplay.setText(zoom_value)

        feedback = postRequest(conn, "camera", {"method": "getStorageInformation", "params": [], "id": 1, "version": "1.0"})
        
        if mode == "movie":
            result = feedback["result"][0][0]["recordableTime"]
            if(result == -1):
                self.memoryDisplay.setText("NO SD-CARD INSERTED!")
            else:
                self.memoryDisplay.setText(str(result) + " MINUTES")
        elif mode == "still":
            result = feedback["result"][0][0]["numberOfRecordableImages"]
            if(result == -1):
                self.memoryDisplay.setText("NO SD-CARD INSERTED!")
            else:
                self.memoryDisplay.setText(str(result) + " IMAGES")
    
    # methods for changing views
    def reinit(self):
        global display
        if(display == 0):
            self.cam1Label.setText("  OPERATOR CAM")
            self.cam2Label.setText("  PILOT CAM")
            self.imgBigDisplay.hide()
            self.picBigDisplay.hide()
            self.imgDisplay.show()
            self.picDisplay.show()
            self.hudDisplay.show()

        if(display == 1):
            self.cam1Label.setText("  OPERATOR CAM")
            self.cam2Label.setText("")
            self.picBigDisplay.hide()
            self.imgDisplay.hide()
            self.picDisplay.hide()
            self.hudDisplay.hide()
            self.imgBigDisplay.show()

        if(display == 2):
            self.cam1Label.setText("  PILOT CAM")
            self.cam2Label.setText("")
            self.imgBigDisplay.hide()
            #self.imgDisplay.hide()
            #self.picDisplay.hide()
            #self.hudDisplay.hide()
            self.picBigDisplay.show()

        print(">>> REINIT SUCCESSFULY!")
        print(">>> MODE NOW: " + str(display))

def setRecMode():
    resp = postRequest(conn, "camera", {"method": "startRecMode", "params": [], "version": "1.0"})

def initValues():
    global mode
    global zoom
    global zoom_value
    if wifi == True:
        setRecMode()
        setMemory()
        zoomPosition()
        
        tmpMode = postRequest(conn, "camera", {"method": "getShootMode", "params": [], "id": 1, "version": "1.0"})
        mode = str(tmpMode["result"][0])
        tmprec = postRequest(conn, "camera", {"method": "getEvent", "params": [False], "id": 4, "version": "1.0"})
        box.statusDisplay.setText(str(tmprec["result"][1]["cameraStatus"]))

    else:
        print(">>> WIFI NOT CONNECTED")
            
def setMemory():
    global mode
    global wifi
    if wifi == True:
        try:
            feedback = postRequest(conn, "camera", {"method": "getStorageInformation", "params": [], "id": 1, "version": "1.0"})

            if mode == "movie":
                result = feedback["result"][0][0]["recordableTime"]
                if(result == -1):
                    box.memoryDisplay.setText("NO SD-CARD INSERTED!")
                else:
                    box.memoryDisplay.setText(str(result) + " MINUTES")
            elif mode == "still":
                result = feedback["result"][0][0]["numberOfRecordableImages"]
                if(result == -1):
                    box.memoryDisplay.setText("NO SD-CARD INSERTED!")
                else:
                    box.memoryDisplay.setText(str(result) + " IMAGES")
        except Exception as ex:
            print (">>> " + ex.message)
    else:
        print(">>> WIFI NOT CONNECTED")

def setModeStill():
    global mode
    global wifi
    if wifi == True:
        resp = postRequest(conn, "camera", {"method": "setShootMode", "params": ["still"], "version": "1.0"})
        mode = "still"
        setMemory()
    else:
        print(">>> WIFI NOT CONNECTED")

def setModeMovie():
    global mode
    global wifi
    if wifi == True:
        resp = postRequest(conn, "camera", {"method": "setShootMode", "params": ["movie"], "version": "1.0"})
        mode = "movie"
        setMemory()
    else:
        print(">>> WIFI NOT CONNECTED")

def takePic():
    global mode
    global rec
    global wifi
    if wifi == True:
        if mode == "still":
            box.statusDisplay.setText("CAPTURING")
            rec = "on"
            resp = postRequest(conn, "camera", {"method": "actTakePicture", "params": [], "version": "1.0"})
            setMemory()
            box.statusDisplay.setText("IDLE")
            rec = "off"


        elif mode == "movie":
            box.statusDisplay.setText("RECORDING")
            if rec == "off":
                resp = postRequest(conn, "camera", {"method": "startMovieRec", "params": [], "version": "1.0"})
                rec = "on"
            elif rec == "on":
                box.statusDisplay.setText("FINISHED")
                resp = postRequest(conn, "camera", {"method": "stopMovieRec", "params": [], "version": "1.0"})
                rec = "off"
                setMemory()
    else:
        print(">>> WIFI NOT CONNECTED")

def zoomIn():
    global wifi
    if wifi == True:
        box.statusDisplay.setText("ZOOM IN")
        resp = postRequest(conn, "camera", {"method": "actZoom", "params": ["in", "start"], "version": "1.0"})
    else:
        print(">>> WIFI NOT CONNECTED")
        
def zoomInStop():
    global wifi
    global zoom
    global zoom_value
    if wifi == True:
        box.statusDisplay.setText("IDLE")
        resp = postRequest(conn, "camera", {"method": "actZoom", "params": ["in", "stop"], "version": "1.0"})
        zoomPosition()
    else:
        print(">>> WIFI NOT CONNECTED")


def zoomOut():
    global wifi
    if wifi == True:
        box.statusDisplay.setText("ZOOM OUT")
        resp = postRequest(conn, "camera", {"method": "actZoom", "params": ["out", "start"], "version": "1.0"})
    else:
        print(">>> WIFI NOT CONNECTED")
        
def zoomOutStop():
    global wifi
    global zoom
    global zoom_value
    if wifi == True:
        box.statusDisplay.setText("IDLE")
        resp = postRequest(conn, "camera", {"method": "actZoom", "params": ["out", "stop"], "version": "1.0"})
        zoomPosition()
    else:
        print(">>> WIFI NOT CONNECTED")

def zoomPosition():
    global wifi
    global zoom
    global zoom_value
    if wifi == True:
        feedback = postRequest(conn, "camera", {"method": "getEvent", "params": [False], "id": 4, "version": "1.0"})
        zoom = feedback["result"][2]["zoomPosition"]
        zoom_value = str(round(zoom*0.333,1)) + " X"
        box.zoomDisplay.setText(zoom_value)
    else:
        print(">>> WIFI NOT CONNECTED")
        
# def main ->            
def main():
    global box
    
    app = QApplication(sys.argv)
    app.setOverrideCursor(QCursor(Qt.BlankCursor))
    app.processEvents()

    palette = QPalette()
    palette.setColor(QPalette.Background,QColor(100, 100, 100))
    
    box = Box(None)
    box.setPalette(palette)
    box.showFullScreen()
    #box.show()
    
    communication = threading.Thread(target = communicationThread)
    communication.start()

    sonyGetBatLevel = threading.Thread(target = sonyGetBatLevelThread)
    sonyGetBatLevel.start()
    
    picamliveview = threading.Thread(target = picamliveviewThread)
    picamliveview.start()
    
    #arduinoSerial = threading.Thread(target = arduinoSerialThread)
    #arduinoSerial.start()

    #test = threading.Thread(target = testingThread)
    #test.start()

    vehicle.add_attribute_listener('location.global_frame', alt_callback)
    vehicle.add_attribute_listener('location.global_relative_frame', land_callback)
    vehicle.add_attribute_listener('system_status', status_callback)
    vehicle.add_attribute_listener('groundspeed', gs_callback)
    vehicle.add_attribute_listener('battery', bat_callback)
    vehicle.add_attribute_listener('attitude', att_callback)
    vehicle.add_attribute_listener('gps_0', sat_callback)
    
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
