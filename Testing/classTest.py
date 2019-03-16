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

print("Initial gain is ", lilbro.driver1.axis0.controller.config.pos_gain)
lilbro.setVelLim(60000)
lilbro.setState(8)

print("Initial velocity limit is ",lilbro.driver1.axis0.controller.config.vel_limit)

while True:
    try:
      # this is a Test for Updating through collabEdit
      #  pos1 = lilbro.getCounts()
      #  print("D1: ",lilbro.getDriver1())
      #  print("Positions: ",lilbro.getCounts())
      #  print("Motor Pos1 = ", pos1[1])
      #  print("Motor currents: ",lilbro.getCurrents())
        
#        lilbro.setGain(50)
#        print("Gain is ", lilbro.driver1.axis0.controller.config.pos_gain)

        print("Error: ",lilbro.isError())

        time.sleep(1.5)

    except (KeyboardInterrupt):
        # Turn off the motors
        # Close the data file

        lilbro.setState(1)
        sys.exit()