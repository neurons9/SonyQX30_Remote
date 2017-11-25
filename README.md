# SonyQX30_Remote
Raspberry Pi 3 Python based App to stream Live Video out from Sony lensstyle QX30 for drone applications
<b>Features</b>
1. Control Sony Lens Camera: Take Picture or Video, Zoom, switch mode
2. See status of Sony Lens Camera: battery, SD Card, mode and zoom factor
3. Live picture feed
4. See status of Autopilot: battery, GPS Quality, artifical horizon, speed, altitude, yaw orientation
5. See live feed from Raspy Cam
6. switch to see both cams at one time, or only Sony or only Raspy Cam
7. experimential Face detection with OpenCV

<b>Ideas for future releases:</b>
1. add precicion landing with Pi Camera face down (moved by a small servo)
2. add object tracking by moving the Sony Gimbal
3. add more control over Sony Camera with a menu controlled by two channels, one to move the pointer and one to manipulate the value

<b>requirements Hardware:</b>
1. Raspberry Pi 3
2. Raspberry Pi Camera
3. Sony DSC QX1/10/30
4. HDMI Monitor

<b>requirements Software:</b>
1. Raspberry Pi Desktop or Ubuntu
2. Python 2.7 (was the max Version for Dronekit 1.x will be Python 3 for future releases)
3. Mavproxy (will be removed as Dronekit 2.0 don't need it)
4. Dronekit 1.5 (will be migrates to 2.0)
5. OpenCV

<b>To do:</b>
1. Migrate to DKPY 2.0
2. remove code for controlling RC-in channels with Arduino Serial connection and monitor channels via mavlink

You can find informations on how to install needed modules on the following pages:

https://www.raspberrypi.org/downloads/

http://python.dronekit.io/1.5.0/

http://ardupilot.github.io/MAVProxy/

https://opencv.org


detailed informations on how to compile openCV on Adrians pages:
https://www.pyimagesearch.com/2017/09/04/raspbian-stretch-install-opencv-3-python-on-your-raspberry-pi/

The pictures below are showing the output of Raspberry Pi. You can connect the HDMI port to an Ammimon Connex or Lightbridge to get both camera feeds transmitted
