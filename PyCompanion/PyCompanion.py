#!/usr/bin/env python2
# coding=utf-8
import sys, json, time
import serial
import threading
import base64, hashlib
import functools

from dronekit import connect, VehicleMode
from PyQt4.QtCore import *
from PyQt4.QtGui import *

# vars ->
lock   = threading.Lock()

global armed
global vehicle

global alt_value
global sat_value
global gs_value
global yaw_value
global bat_cur
global bat_volt
global bat_value
global zenlogo

global roll
global nick
global yaw
global alt
global sat

global appWidth
global appHeight
global imgHeight

global batLevel

output     = "HDMI" #HDMI/RGB

armed      = 0

sat_value  = "0 SAT. No Fix"
alt_value  = "0 m"
gs_value   = "0 km/h"
yaw_value  = "0"
bat_volt   = "0 V"
bat_cur    = "0 A"
bat_value  = "0 %"

roll = 0
nick = 0
yaw  = 0
alt  = 0
sat  = 0

batNumer  = 0
batDenom  = 4
batLevel  = 0


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
imgSatOn       = "/home/pi/Documents/QX30/img/imgSatOn.png"
imgSatOff      = "/home/pi/Documents/QX30/img/imgSatOff.png"
imgBat000      = "/home/pi/Documents/QX30/img/batt_000.png"
imgBat020      = "/home/pi/Documents/QX30/img/batt_020.png"
imgBat040      = "/home/pi/Documents/QX30/img/batt_040.png"
imgBat060      = "/home/pi/Documents/QX30/img/batt_060.png"
imgBat080      = "/home/pi/Documents/QX30/img/batt_080.png"
imgBat100      = "/home/pi/Documents/QX30/img/batt_100.png"
imgBat999      = "/home/pi/Documents/QX30/img/batt_999.png"
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


# initialize vehicle
vehicle = connect('/dev/ttyUSB0', wait_ready=False, baud=57600, heartbeat_timeout=60)

# dynamic labels ->
class SatAnimation(QLabel):
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


class BatAnimation(QLabel):
    def __init__(self):
        QLabel.__init__(self)

    def paintEvent(self, event):
        global batLevel
        if batLevel <= 15:
            imgBat = imgBat000
        elif batLevel > 15 and batLevel <= 35:
            imgBat = imgBat020
        elif batLevel > 35 and batLevel <= 55:
            imgBat = imgBat040
        elif batLevel > 55 and batLevel <= 75:
            imgBat = imgBat060
        elif batLevel > 75 and batLevel <= 95:
            imgBat = imgBat080
        elif batLevel > 95:
            imgBat = imgBat100

        img = QImage(imgBat)
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
    box.satLabel.setText(sat_value)

def alt_callback(self, attr_name, value):
    global alt_value
    global alt
    alt = value.alt
    alt_value = str(round(alt,1)) + " m"
    box.altLabel.setText(alt_value)

def status_callback(self, attr_name, value):
    global armed
    armed = value.armed

def gs_callback(self, attr_name, value):
    global gs_value
    gs_value = str(round(value,1)) + " km/h"
    box.speedLabel.setText(gs_value)

def bat_callback(self, attr_name, value):
    global bat_cur
    global bat_volt
    global bat_value
    global batLevel
    if value.current:
        bat_cur = str(round(value.current,1)) + " A"
    else:
        bat_cur = "0.0 A"
    if value.level:
        bat_value = str(round(value.level,1)) + " %"
        batLevel  = value.level
    else:
        bat_value = "0 %"
        batLevel  = 0
    bat_volt = bat_value + "  " + str(round(value.voltage,1)) + " V"
    box.batLabel.setText(bat_volt)
    box.curLabel.setText(bat_cur)

def att_callback(self, attr_name, value):
    global roll
    global nick
    global yaw
    global yaw_value
    roll = round(value.roll,1)
    nick = round(value.pitch,1)
    yaw =  round(value.yaw,1)
    yaw_value  = str(round(yaw*56.25,1)) + QString.fromUtf8("° ")
    box.yawLabel.setText(yaw_value)

# main GUI ->
class Box(QDialog):

    def __init__(self, parent=None, margin=-10, spacing=-10):
        super(Box, self).__init__(parent)
        self.initUI()

        self.setContentsMargins(-11,-11,-11,-12)


    def initUI(self):
        global mode

        self.batLabel = QLabel("0%  0.0 V ")
        self.batLabel.setStyleSheet(fontBig2)

        self.curIcon = QLabel()
        self.curIcon.setPixmap(QPixmap(imgCurrent))

        self.curLabel = QLabel("0.0 A ")
        self.curLabel.setStyleSheet(fontBig2)

        self.satLabel = QLabel("0 SATs")
        self.satLabel.setStyleSheet(fontBig2)

        self.altIcon = QLabel()
        self.altIcon.setPixmap(QPixmap(imgAlt))

        self.altLabel = QLabel("0.0 m ")
        self.altLabel.setStyleSheet(fontBig2)

        self.distLabel = QLabel("DIST ")
        self.distLabel.setStyleSheet(fontSmall)
        self.distLabel.setAlignment(Qt.AlignRight|Qt.AlignVCenter)

        self.distLabel = QLabel("0 m ")
        self.distLabel.setStyleSheet(fontBig2)

        self.speedIcon = QLabel()
        self.speedIcon.setPixmap(QPixmap(imgSpeed))

        self.speedLabel = QLabel("0.0 km/h ")
        self.speedLabel.setStyleSheet(fontBig2)

        self.yawIcon = QLabel()
        self.yawIcon.setPixmap(QPixmap(imgYaw))

        self.yawLabel = QLabel(QString.fromUtf8("0° "))
        self.yawLabel.setStyleSheet(fontBig2)

        self.zen = QLabel()
        self.zen.setPixmap(QPixmap(zenlogo))
        self.zen.setAlignment(Qt.AlignCenter|Qt.AlignVCenter)

        self.hudDisplay  = HudDisplay()

        self.hudDisplay.setMaximumSize(appWidth/2, imgHeight)
        self.hudDisplay.setAlignment(Qt.AlignCenter|Qt.AlignVCenter)

        satAnim  = SatAnimation()
        batAnim  = BatAnimation()

        self.main = QGridLayout()
        self.main.setSpacing(0)

        self.grid = QGridLayout()
        self.grid.setSpacing(0)

        self.grid.addWidget(self.hudDisplay,1,1)

        self.grid.setRowMinimumHeight(1,imgHeight)
        self.grid.setColumnMinimumWidth(0,appWidth/2)
        self.grid.setColumnMinimumWidth(1,appWidth/2)

        self.osd1 = QGridLayout()
        self.osd1.setColumnMinimumWidth(0,55)
        self.osd1.setColumnMinimumWidth(1,55)
        self.osd1.setColumnMinimumWidth(2,290)
        self.osd1.setRowMinimumHeight(0,50)
        self.osd1.setRowMinimumHeight(1,50)
        self.osd1.setRowMinimumHeight(2,50)

        self.osd2 = QGridLayout()
        self.osd2.addWidget(batAnim,0,0)
        self.osd2.addWidget(self.batLabel,0,1)
        self.osd2.addWidget(self.curIcon,0,2)
        self.osd2.addWidget(self.curLabel,0,3)
        self.osd2.addWidget(satAnim,1,0)
        self.osd2.addWidget(self.satLabel,1,1)
        self.osd2.addWidget(self.altIcon,1,2)
        self.osd2.addWidget(self.altLabel,1,3)
        self.osd2.addWidget(self.speedIcon,2,0)
        self.osd2.addWidget(self.speedLabel,2,1)
        self.osd2.addWidget(self.yawIcon,2,2)
        self.osd2.addWidget(self.yawLabel,2,3)

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

    vehicle.add_attribute_listener('location.global_frame', alt_callback)
    vehicle.add_attribute_listener('system_status', status_callback)
    vehicle.add_attribute_listener('groundspeed', gs_callback)
    vehicle.add_attribute_listener('battery', bat_callback)
    vehicle.add_attribute_listener('attitude', att_callback)
    vehicle.add_attribute_listener('gps_0', sat_callback)

    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
