import time as time
# ODrive
import odrive
from odrive.enums import *
import math
# Joystick
import os, struct, array
from fcntl import ioctl
import threading
# Shell commands
from subprocess import call
import sys
import matplotlib.pyplot as plt
import numpy as np
import robot
import control
import globals

#drive1 is top right driver
#drive2 is bottom right driver

saveVar = 0

globals.initialize()
lilbro = robot.robot()
ctrl = control.control()

print("Finding an ODrive...")
#my_drive1 = odrive.find_any("usb","206237793548")
#my_drive2 = odrive.find_any("usb","388937803437")
lilbro.findDrivers(206237793548,388937803437)

readJSThrd = threading.Thread(target=ctrl.ctrl)
readJSThrd.daemon = True
readJSThrd.start()

#while(globals.mode == 0):
#  print("In loop ",globals.reqState)
#  if(globals.reqState == 8):
#    lilbro.setState(globals.reqState)
#    globals.mode = 1
#    break

while True:
    try:
        print("Error: ",lilbro.isError())

        lilbro.writeToFile(saveVar)

        time.sleep(1)

    except (KeyboardInterrupt):
        # Turn off the motors
        # Close the data file

        lilbro.setState(1)
        sys.exit()