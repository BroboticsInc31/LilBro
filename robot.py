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

    def setState(self,state):
        self.driver1.axis0.requested_state = state
        self.driver1.axis1.requested_state = state

        self.driver2.axis0.requested_state = state
        self.driver2.axis1.requested_state = state

    def setGain(self,newGain):
        self.driver1.axis0.controller.config.pos_gain = newGain
        self.driver1.axis1.controller.config.pos_gain = newGain

        self.driver2.axis0.controller.config.pos_gain = newGain
        self.driver2.axis1.controller.config.pos_gain = newGain

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

    def setVelLim(self,velLim):
        self.driver1.axis0.controller.config.vel_limit = velLim
        self.driver1.axis1.controller.config.vel_limit = velLim

        self.driver2.axis0.controller.config.vel_limit = velLim
        self.driver2.axis1.controller.config.vel_limit = velLim

    def setCurLim(self,curLim):
        self.driver1.axis0.motor.config.current_lim = curLim
        self.driver1.axis1.motor.config.current_lim = curLim

        self.driver2.axis0.motor.config.current_lim = curLim
        self.driver2.axis1.motor.config.current_lim = curLim

    def getCounts(self):
        return [self.driver1.axis0.encoder.pos_estimate,self.driver1.axis1.encoder.pos_estimate,self.driver2.axis0.encoder.pos_estimate,self.driver2.axis1.encoder.pos_estimate];
        
    def getCurrents(self):
        return [self.driver1.axis0.motor.current_control.Iq_measured,self.driver1.axis1.motor.current_control.Iq_measured,self.driver2.axis0.motor.current_control.Iq_measured,self.driver2.axis1.motor.current_control.Iq_measured];

    def getBusVoltage(self):
        return [self.driver1.vbus_voltage,self.driver2.vbus_voltage];

    def getTemp(self):
        return [self.driver1.axis0.get_temp(),self.driver1.axis1.get_temp(),self.driver2.axis0.get_temp()],self.driver2.axis1.get_temp()];

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

    #    def writeToFile(self)
  #        startTime = time.time()
  #        motorPos = self.getCounts()

  #      np.savetxt('startSequence.txt',np.c_[array2,array3],fmt="%.3f %.3f")
  
    # * SetTunings(...)*************************************************************
    # * This function allows the controller's dynamic performance to be adjusted.
    # * it's called automatically from the constructor, but tunings can also
    # * be adjusted on the fly during normal operation
    # ******************************************************************************/

    #def SetTunings(self, Kp, Ki, Kd, POn):
    #    if (Kp < 0) or (Ki < 0) or (Kd < 0):
    #        return

    #    self.pOn = POn
    #    self.pOnE = (POn == P_ON_E)

    #    self.dispKp = Kp
    #    self.dispKi = Ki
    #    self.dispKd = Kd

    #    SampleTimeInSec = self.SampleTime

    #    self.kp = Kp
    #    self.ki = Ki * SampleTimeInSec
    #    self.kd = Kd / SampleTimeInSec

    #    if (self.controllerDirection == REVERSE):
    #        self.kp = -self.kp
    #        self.ki = -self.ki
    #        self.kd = -self.kd

    ## * SetTunings(...)*************************************************************
    # * Set Tunings using the last-rembered POn setting
    # ******************************************************************************
    # def SetTunings(self, Kp, Ki, Kd):
    #     SetTunings(Kp, Ki, Kd, self.pOn)

    # * SetSampleTime(...) *********************************************************
    # * sets the period, in Milliseconds, at which the calculation is performed
    # ******************************************************************************
    #def SetSampleTime(self, NewSampleTime):
    #    NewSampleTime = float(NewSampleTime)

    #    if NewSampleTime > 0.0:
    #        ratio = NewSampleTime / self.SampleTime

    #        self.ki *= ratio
    #        self.kd /= ratio
    #        self.SampleTime = NewSampleTime

    ## * SetOutputLimits(...)****************************************************
    # *     This function will be used far more often than SetInputLimits.  while
    # *  the input to the controller will generally be in the 0-1023 range (which is
    # *  the default already,)  the output will be a little different.  maybe they'll
    # *  be doing a time window and will need 0-8000 or something.  or maybe they'll
    # *  want to clamp it from 0-125.  who knows.  at any rate, that can all be done
    # *  here.
    # **************************************************************************
    #def SetOutputLimits(self, Min,Max):
    #    if Min > Max:
    #        return

    #    self.outMin = Min
    #    self.outMax = Max

    #    if self.inAuto:
    #        if self.output > self.outMax:
    #            self.output = self.outMax
    #        elif self.output < self.outMin:
    #            self.output = self.outMin

    #        if self.outputSum > self.outMax:
    #            self.outputSum = self.outMax
    #        elif self.outputSum < self.outMin:
    #            self.outputSum = self.outMin

    ## * SetMode(...)****************************************************************
    # * Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
    # * when the transition from manual to auto occurs, the controller is
    # * automatically initialized
    # ******************************************************************************
    #def SetMode(self, Mode):
    #    newAuto = (Mode == AUTOMATIC)

    #    if newAuto and (self.inAuto == False):
    #        self.Initialize()

    #    self.inAuto = newAuto

    # * Initialize()****************************************************************
    # *  does all the things that need to happen to ensure a bumpless transfer
    # *  from manual to automatic mode.
    # ******************************************************************************
    #def Initialize(self):
    #    self.outputSum = self.output

    #    if self.outputSum > self.outMax:
    #        self.outputSum = self.outMax
    #    elif self.outputSum < self.outMin:
    #        self.outputSum = self.outMin

    ## * SetControllerDirection(...)*************************************************
    # * The PID will either be connected to a DIRECT acting process (+Output leads
    # * to +Input) or a REVERSE acting process(+Output leads to -Input.)  we need to
    # * know which one, because otherwise we may increase the output when we should
    # * be decreasing.  This is called from the constructor.
    # ******************************************************************************
    #def SetControllerDirection(self, Direction):
    #    if self.inAuto and (Direction != controllerDirection):
    #        self.kp = -self.kp
    #        self.ki = -self.ki
    #        self.kd = -self.kd
    #    self.controllerDirection = Direction

    #def SetSetpoint(self, newSetpoint):
    #    self.setpoint = newSetpoint

    ## * Status Funcions*************************************************************
    # * Just because you set the Kp=-1 doesn't mean it actually happened.  these
    # * functions query the internal state of the PID.  they're here for display
    # * purposes.  this are the functions the PID Front-end uses for example
    # ******************************************************************************
    #def GetKp(self):
    #    return self.dispKp
    #def GetKi(self):
    #    return self.dispKi
    #def GetKd(self):
    #    return self.dispKd
    #def GetMode():
    #    return (MANUAL, AUTOMATIC)[self.inAuto == True]
    #def GetDirection():
    #    return self.ControllerDirection
