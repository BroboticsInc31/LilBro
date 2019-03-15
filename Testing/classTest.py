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

#drive1 is top right driver
#drive2 is bottom right driver

lilbro = robot.robot()

print("Finding an ODrive...")
#my_drive1 = odrive.find_any("usb","206237793548")
#my_drive2 = odrive.find_any("usb","388937803437")
lilbro.findDrivers(206237793548,388937803437)

lilbro.setState(1)

while True:
    try:
        print("D1: ",lilbro.getDriver1())
        time.sleep(1)

    except (KeyboardInterrupt):
        # Turn off the motors
        # Close the data file
        testdata.close()
        sys.exit()


