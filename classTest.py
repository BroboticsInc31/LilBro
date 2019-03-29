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
import odrive

#drive1 is top right driver
#drive2 is bottom right driver

saveVar = 0

globals.initialize()
lilbro = robot.robot()
ctrl = control.control()

print("Finding an ODrive...")
#my_drive1 = odrive.find_any("usb","206237793548")
#my_drive2 = odrive.find_any("usb","388937803437")
#lilbro.findDrivers(206237793548,388937803437)
driver1 = odrive.find_any()

driver1.axis0.requested_state = 1
driver1.axis1.requested_state = 1

readJSThrd = threading.Thread(target=ctrl.ctrl)
readJSThrd.daemon = True
readJSThrd.start()

testPos1 = lilbro.toMotor(lilbro.toCount(360))
posArray1 = np.linspace(0,testPos1,150)
posArray2 = np.linspace(testPos1,0,150)

#while(globals.mode == 0):
#  print("In loop ",globals.reqState)
#  if(globals.reqState == 8):
#    lilbro.setState(globals.reqState)
#    globals.mode = 1
#    break
print("Press O to Enter Closed Loop Mode")
while(driver1.axis0.current_state != 8):
    if(globals.reqState == 8):
        driver1.axis0.requested_state = globals.reqState
        driver1.axis1.requested_state = globals.reqState
        print("RAINBOW NIII")
    print("Mode is ",driver1.axis0.current_state," ,",driver1.axis1.current_state)
    time.sleep(0.1)

while True:
    try:
#        print("Error: ",lilbro.isError())

#        lilbro.writeToFile(saveVar)

        if(globals.reqState == 1):
            driver1.axis0.requested_state = 1
            driver1.axis1.requested_state = 1
            globals.sweepOn = 0

        if(globals.sweepOn == 1 and globals.reqState == 8):
            for i in range(len(posArray1)):
                if(globals.sweepOn == 0 or globals.reqState == 1):
                    break
                
                driver1.axis0.controller.pos_setpoint = posArray1[i]
                driver1.axis1.controller.pos_setpoint = posArray1[i]

                time.sleep(0.01)

            for j in range(len(posArray2)):
                if(globals.sweepOn == 0 or globals.reqState == 1):
                    break
                
                driver1.axis0.controller.pos_setpoint = posArray2[j]
                driver1.axis1.controller.pos_setpoint = posArray2[j]

                time.sleep(0.01)
            

    except (KeyboardInterrupt):
        # Turn off the motors
        # Close the data file

        driver1.axis0.requested_state = 1
        driver1.axis1.requested_state = 1
        
#        lilbro.setState(1)
        sys.exit()