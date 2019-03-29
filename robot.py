#!usr/bin/python
# -----------------------------------
# An Odrive class used to control lil'Bro; an open source quadrupedal robot for UTSA's Robotics and Motion Laboratory
# 
#
# @authors Steven Farra, Emiliano Rodriguez 
# @version 1.0
# @since 2019-03-11
# -----------------------------------

from __future__ import division
import time
import odrive
import math
import numpy as np
import globals

#AUTOMATIC  = 1
#MANUAL = 0
#DIRECT = 0
#REVERSE = 1
#P_ON_M = 0
#P_ON_E = 1

class robot():
    """This is the constructor for the robot class.

    In this constructor, the proportional gain, current limit,
    velocity limit, and drivers are initialized and delclared.

    @params self
    @return Unused

    """

    def __init__(self):

        self.pGain = 20
        self.driverCurLim = 50
        self.driverVelLim = 60000

        self.driver1 = 0
        self.driver2 = 0
        self.driver3 = 0
        self.driver4 = 0

        global timeArray
        timeArray = []
        global motorPos
        motorPos = []
        global motor1
        motor1 = []
    
    """ findDrivers finds Odrive motor drivers given their serial number in main class

    This method uses the Serial number of an odrive motor driver that is 
    connected via USB. 
    
    @params driverID1,driverID2 
    @return Unused

    """

    def findDrivers(self, driverID1, driverID2):
        self.driver1 = odrive.find_any("usb",str(driverID1))
        self.driver2 = odrive.find_any("usb",str(driverID2))
        now = time.time()
    
    def getDriver1(self):
        return self.driver1.serial_number
        
    def getDriver2(self):
        return self.driver2.serial_number

    def setState(self,xdriver,xstate):
        xdriver.axis0.requested_state = xstate
        xdriver.axis1.requested_state = xstate
    
    def setStates(self,state):
        self.setState(self.driver1,state)
        self.setState(self.driver2,state)

    def setGain(self,xdriver,xgain):
        xdriver.axis0.controller.config.pos_gain = xgain
        xdriver.axis1.controller.config.pos_gain = xgain

    def setGains(self,newGain):
        self.setGain(self.driver1,newGain)
        self.setGain(self.driver2,newGain)

    def addGain(self,incGain):
        self.driver1.axis0.controller.config.pos_gain += incGain
        self.driver1.axis1.controller.config.pos_gain += incGain

        self.driver2.axis0.controller.config.pos_gain += incGain
        self.driver2.axis1.controller.config.pos_gain += incGain

    def isError(self):
        if (self.driver1.axis0.error != 0 or self.driver1.axis0.error != 0 or self.driver2.axis0.error != 0 or self.driver2.axis1.error != 0):
            return 1
        else:
            return 0


    def setVelLim(self,xdriver,xlim):
        xdriver.axis0.controller.config.vel_limit = xlim
        xdriver.axis1.controller.config.vel_limit = xlim

    def setVelLims(self,velLim):
        self.setVelLim(self.driver1,velLim)
        self.setVelLim(self.driver2,velLim)


    def setCurLim(self,xdriver,xlim):
        xdriver.axis0.motor.config.current_lim = xlim
        xdriver.axis1.motor.config.current_lim = xlim

    def setCurLims(self,curLim):
        self.setCurLim(self.driver1,curLim)
        self.setCurLim(self.driver2,curLim)


    def getCount(self,driver):
        return [self.driver.axis0.encoder.pos_estimate,self.driver.axis1.encoder.pos_estimate];

    def getCounts(self):
        return [self.driver1.axis0.encoder.pos_estimate,self.driver1.axis1.encoder.pos_estimate,self.driver2.axis0.encoder.pos_estimate,self.driver2.axis1.encoder.pos_estimate];


    def getCurrent(self,driver):
        return [self.driver.axis0.motor.current_control.Iq_measured,self.driver.axis1.motor.current_control.Iq_measured]

    def getCurrents(self):
        return [self.driver1.axis0.motor.current_control.Iq_measured,self.driver1.axis1.motor.current_control.Iq_measured,self.driver2.axis0.motor.current_control.Iq_measured,self.driver2.axis1.motor.current_control.Iq_measured];


    def getBusVoltage(self):
        return [self.driver1.vbus_voltage,self.driver2.vbus_voltage];

    def getTemp(self):
        return [self.driver1.axis0.get_temp(),self.driver1.axis1.get_temp(),self.driver2.axis0.get_temp(),self.driver2.axis1.get_temp()];


    def writeToFile(self,saved):
        if(globals.dataOn == 1):
            motorPos = self.getCounts()
            timeArray.append(time.time() - globals.startTime)
            motor1.append(motorPos[0])
        elif(globals.dataOn == 0 and saved == 0):
            np.savetxt('ClassPositions.txt',np.c_[timeArray,motor1],fmt="%.3f %.3f")
            saved = 1


    def toDeg(self,counts):
        return 360*(counts)/8192

    def toCount(self,degrees):
        return 8192*(degrees)/360

    def toLeg(self,counts):
        return counts*(1/5)

    def toMotor(self,counts):
        return counts*5

    def setPos(self,driver,pos):
        driver.axis0.controller.pos_setpoint = pos
        driver.axis1.controller.pos_setpoint = pos

    def takeStep(self,driver,pos):
        driver.axis0.controller.pos_setpoint = pos
        driver.axis1.controller.pos_setpoint = pos

    # def checkStep(self,driver,deg):
    #     if()
  

    def rotation(self,theta):
        return R = np.array([[math.cos(math.radians(theta)),-math.sin(math.radians(theta))],[math.sin(math.radians(theta)),math.cos(math.radians(theta))]])

    def symmetric(self,alpha1,alpha2,l1,l2):
        base = [0,0]
        leftshoulder = np.array([l1*math.cos(math.radians(alpha2)),l1*math.sin(math.radians(alpha2))])
        rightshoulder = np.array([l1*math.cos(math.radians(alpha2)),-l1*math.sin(math.radians(alpha2))])
        foot = np.array([l1*math.cos(math.radians(alpha2))+math.sqrt((l2**2)-(l1**2)*(math.sin(math.radians(alpha2)))**2),0])

        R = rotation(alpha1)
        leftshoulder = R.dot(leftshoulder)
        rightshoulder = R.dot(rightshoulder)
        foot = R.dot(foot)

        return [base,leftshoulder,rightshoulder,foot]; 
