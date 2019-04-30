#!usr/bin/python
# -----------------------------------
# An Odrive class used to control lil'Bro; an open source quadrupedal robot for UTSA's Robotics and Motion Laboratory
# 
#
# @authors Steven Farra, Emiliano Rodriguez 
# @version 1.0
# @since 2019-03-11
#
#
#
#
# @section license License
#
# This file is a part of the LilBro library 
#
# BroboticsInc LilBro is free software: you can redistribute it and/or modify it under the
# terms of the GNU General Public License as published by the Free Software Foundation,
# either version 3 of the License, or (at your option) any later version
#
# BroboticsInc LilBro is distributed in hope that the software will be useful, but 
# WITHOUT WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR 
# A PARTICULAR PURPOSE. See the GNU General Public License for more details. 
#
# You should have received a copy of the GNU General Public License along with LilBro. 
# If not, see 
# <http://www.gnu.org/licenses/>
#
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

        global n, nh
        n = 32
        nh = 7
        self.leftShoulderx = np.array([np.zeros(n),np.zeros(n),np.zeros(n),np.zeros(n)])
        self.rightShoulderx = np.array([np.zeros(n),np.zeros(n),np.zeros(n),np.zeros(n)])
        self.leftShouldery = np.array([np.zeros(n),np.zeros(n),np.zeros(n),np.zeros(n)])
        self.rightShouldery = np.array([np.zeros(n),np.zeros(n),np.zeros(n),np.zeros(n)])

        self.leftShoulderxh = np.array([np.zeros(nh),np.zeros(nh),np.zeros(nh),np.zeros(nh)])
        self.rightShoulderxh = np.array([np.zeros(nh),np.zeros(nh),np.zeros(nh),np.zeros(nh)])
        self.leftShoulderyh = np.array([np.zeros(nh),np.zeros(nh),np.zeros(nh),np.zeros(nh)])
        self.rightShoulderyh = np.array([np.zeros(nh),np.zeros(nh),np.zeros(nh),np.zeros(nh)])

        global timeArray
        timeArray = []
        global motorPos, motorVel, motorCur
        motorPos = []
        motorVel = []
        motorCur = []
        global motors0, motors1, motors2, motors3, motors4, motors5, motors6, motors7
        motors0, motors1, motors2, motors3, motors4, motors5, motors6, motors7 = [], [], [], [], [], [], [], []
        global motorsVel0, motorsVel1, motorsVel2, motorsVel3, motorsVel4, motorsVel5, motorsVel6, motorsVel7 
        motorsVel0, motorVel1, motorsVel2, motorsVel3, motorsVel4, motorsVel5, motorsVel6, motorsVel7 = [], [], [], [], [], [], [], []
        global motorsCur0, motorsCur1, motorsCur2, motorsCur3, motorsCur4, motorsCur5, motorsCur6, motorsCur7
        motorsCur0, motorCur1, motorsCur2, motorsCur3, motorsCur4, motorsCur5, motorsCur6, motorsCur7 = [], [], [], [], [], [], [], []

        global offset1, offset2, offset3, offset4
        offset1 = [18.7,21.5]
        offset2 = [35,15]
        offset3 = [31.5,49.3]
        offset4 = [20.5,45.5]

        global ls1, ls2, as1, as2, t1, t2, theta1, theta2, theta1_t, theta2_t, theta1_ht, theta2_ht, t1_th, t2_th, theta1f, theta2f, theta1_b, theta2_b
        ls1 = np.array([0.17,0,0,-35,262.5,-656.25,546.875])
        ls2 = np.array([2.73,-28.8,132,-315,412.5,-281.25,78.125])
        as1 = np.array([0.2618,0,0,-81.81,306.8,-306.8,1.33*(10**-11)])
        as2 = np.array([-16.493,157.08,-589,1063.6,-920.4,306.8,-2*(10**-10)])

        global ls1_t, ls2_t, as1_t, as2_t, t1_trot, t2_trot, l_down, ls1_t2, ls2_t2, as1_t2, as2_t2
        #ls1_t = np.array([0.175,0,0,-40,300,-750,625])
        #ls2_t = np.array([0.175,-1.847*(10**-14),4.75*(10**-13),-3.356*(10**-13),9.159*(10**-13),-2.607*(10**-13),-1.903*(10**-13)])
        #as1_t = np.array([0.1309,0,0,-40.91,153.3981,-153.3981,6.67*(10**-12)])
        #as2_t = np.array([-8.2467,78.5398,-294.52,531.78,-460.19,153.40,-9.998*(10**-11)])
        l_down = np.linspace(0.135,0.175,nh/4)
        
        ls1_t = np.array([0.148,0,0,-13,97.5,-243.75,203.125])
        ls2_t = np.array([2.708,-28.8,132,-315,412.5,-281.25,78.125])
        as1_t = np.array([0.1309*2,0,0,-40.91*2,153.3981*2,-153.3981*2,6.67*(10**-12)*2])
        as2_t = np.array([-8.2467*2,78.5398*2,-294.52*2,531.78*2,-460.19*2,153.40*2,-9.998*(10**-11)*2])
    
        ls1_t2 = np.array([0.148,0,0,-13,97.5,-243.75,203.125])
        ls2_t2 = np.array([2.708,-28.8,132,-315,412.5,-281.25,78.125])
        as1_t2 = np.array([0.1309*2,0,0,-40.91*2,153.3981*2,-153.3981*2,6.67*(10**-12)*2])
        as2_t2 = np.array([-8.2467*2,78.5398*2,-294.52*2,531.78*2,-460.19*2,153.40*2,-9.998*(10**-11)*2])

        #ls1_t2 = np.array([0.185,0,0,-40,300,-750,625])
        #ls2_t2 = np.array([0.185,4.178*(10**-14),-8.69*(10**-14),-4.309*(10**-13),-1.806*(10**-13),-5.214*(10**-14),-2.20*(10**-13)])
        #as1_t2 = np.array([0.1309,0,0,-40.91,153.3981,-153.3981,6.67*(10**-12)])
        #as2_t2 = np.array([-8.2467,78.5398,-294.52,531.78,-460.19,153.40,-9.998*(10**-11)])
        
        t1 = np.linspace(0,0.2,n/4)
        t2 = np.linspace(0.2,0.8,3*n/4)
        
        t1_trot = np.linspace(0,0.4,n/2)
        t2_trot = np.linspace(0.4,0.8,n/2)

        t1_th = np.linspace(0,0.4,nh/2)
        t2_th = np.linspace(0.4,0.8,nh)

        theta1 = np.array([np.zeros(n),np.zeros(n),np.zeros(n),np.zeros(n)])
        theta2 = np.array([np.zeros(n),np.zeros(n),np.zeros(n),np.zeros(n)])

        theta1f = np.array([np.zeros(n),np.zeros(n),np.zeros(n),np.zeros(n)])
        theta2f = np.array([np.zeros(n),np.zeros(n),np.zeros(n),np.zeros(n)])

        theta1_t = np.array([np.zeros(n),np.zeros(n),np.zeros(n),np.zeros(n)])
        theta2_t = np.array([np.zeros(n),np.zeros(n),np.zeros(n),np.zeros(n)])

        theta1_b = np.array([np.zeros(n),np.zeros(n),np.zeros(n),np.zeros(n)])
        theta2_b = np.array([np.zeros(n),np.zeros(n),np.zeros(n),np.zeros(n)])

        theta1_ht = np.array([np.zeros(nh),np.zeros(nh),np.zeros(nh),np.zeros(nh)])
        theta2_ht = np.array([np.zeros(nh),np.zeros(nh),np.zeros(nh),np.zeros(nh)])
    
    """ findDrivers finds Odrive motor drivers given their serial number in main class

    This method uses the Serial number of an odrive motor driver that is 
    connected via USB. 
    
    @params driverID1,driverID2 
    @return Unused

    """

    def findDrivers(self, driverID1, driverID2, driverID3, driverID4):
        self.driver1 = odrive.find_any("usb",str(driverID1))
        self.driver2 = odrive.find_any("usb",str(driverID2))
        self.driver3 = odrive.find_any("usb",str(driverID3))
        self.driver4 = odrive.find_any("usb",str(driverID4))
        print('Found drivers!')
    
    def getDriver1(self):
        return self.driver1.serial_number
        
    def getDriver2(self):
        return self.driver2.serial_number

    def getDriver3(self):
        return self.driver3.serial_number
        
    def getDriver4(self):
        return self.driver4.serial_number

    def setState(self,xdriver,xstate):
        xdriver.axis0.requested_state = xstate
        xdriver.axis1.requested_state = xstate
    
    def setStates(self,state):
        self.setState(self.driver1,state)
        self.setState(self.driver2,state)
        self.setState(self.driver3,state)
        self.setState(self.driver4,state)

    def setPGain(self,xdriver,xgain):
        xdriver.axis0.controller.config.pos_gain = xgain
        xdriver.axis1.controller.config.pos_gain = xgain

    def setPGains(self,newGain):
        self.setPGain(self.driver1,newGain)
        self.setPGain(self.driver2,newGain)
        self.setPGain(self.driver3,newGain)
        self.setPGain(self.driver4,newGain)

    def setVelGain(self,xdriver,xgain):
        xdriver.axis0.controller.config.vel_gain = xgain
        xdriver.axis1.controller.config.vel_gain = xgain

    def setVelGains(self,newGain):
        self.setVelGain(self.driver1,newGain)
        self.setVelGain(self.driver2,newGain)
        self.setVelGain(self.driver3,newGain)
        self.setVelGain(self.driver4,newGain)

    def setVelIntGain(self,xdriver,xgain):
        xdriver.axis0.controller.config.vel_integrator_gain = xgain
        xdriver.axis1.controller.config.vel_integrator_gain = xgain

    def setVelIntGains(self,newGain):
        self.setVelIntGain(self.driver1,newGain)
        self.setVelIntGain(self.driver2,newGain)
        self.setVelIntGain(self.driver3,newGain)
        self.setVelIntGain(self.driver4,newGain)

    def getPGain(self,axis):
        return axis.controller.config.pos_gain;

    def getVelGain(self):
        return self.driver1.axis0.controller.config.vel_gain;

    def getVelIntGain(self):
        return self.driver1.axis0.controller.config.vel_integrator_gain;

    def addPGain(self,driver,incGain):
        driver.axis0.controller.config.pos_gain += incGain
        driver.axis1.controller.config.pos_gain += incGain

    def addPGains(self,incGain):
        self.addPGain(self.driver1,incGain)
        self.addPGain(self.driver2,incGain)
        self.addPGain(self.driver3,incGain)
        self.addPGain(self.driver4,incGain)

    def addVelGain(self,incGain):
        self.driver1.axis0.controller.config.vel_gain += incGain
        self.driver1.axis1.controller.config.vel_gain += incGain

        self.driver2.axis0.controller.config.vel_gain += incGain
        self.driver2.axis1.controller.config.vel_gain += incGain

        self.driver3.axis0.controller.config.vel_gain += incGain
        self.driver3.axis1.controller.config.vel_gain += incGain

        self.driver4.axis0.controller.config.vel_gain += incGain
        self.driver4.axis1.controller.config.vel_gain += incGain

    def isError(self):
        if (self.driver1.axis0.error != 0 or self.driver1.axis0.error != 0 or self.driver2.axis0.error != 0 or self.driver2.axis1.error != 0 or self.driver3.axis0.error != 0 or self.driver3.axis1.error != 0 or self.driver4.axis0.error != 0 or self.driver4.axis1.error != 0):
            return 1
        else:
            return 0


    def setVelLim(self,xdriver,xlim):
        xdriver.axis0.controller.config.vel_limit = xlim
        xdriver.axis1.controller.config.vel_limit = xlim

    def setVelLims(self,velLim):
        self.setVelLim(self.driver1,velLim)
        self.setVelLim(self.driver2,velLim)
        self.setVelLim(self.driver3,velLim)
        self.setVelLim(self.driver4,velLim)


    def setCurLim(self,xdriver,xlim):
        xdriver.axis0.motor.config.current_lim = xlim
        xdriver.axis1.motor.config.current_lim = xlim

    def setCurLims(self,curLim):
        self.setCurLim(self.driver1,curLim)
        self.setCurLim(self.driver2,curLim)
        self.setCurLim(self.driver3,curLim)
        self.setCurLim(self.driver4,curLim)

    def setTrajParms(self,driver,vlim,alim,dlim,A):
        driver.axis0.trap_traj.config.vel_limit = vlim
        driver.axis0.trap_traj.config.accel_limit = alim
        driver.axis0.trap_traj.config.decel_limit = dlim
        driver.axis0.trap_traj.config.A_per_css = A

        driver.axis1.trap_traj.config.vel_limit = vlim
        driver.axis1.trap_traj.config.accel_limit = alim
        driver.axis1.trap_traj.config.decel_limit = dlim
        driver.axis1.trap_traj.config.A_per_css = A

    def setTrajAll(self,vlim,alim,dlim,A):
        self.setTrajParms(self.driver1,vlim,alim,dlim,A)
        self.setTrajParms(self.driver2,vlim,alim,dlim,A)
        self.setTrajParms(self.driver3,vlim,alim,dlim,A)
        self.setTrajParms(self.driver4,vlim,alim,dlim,A)

    def addAccelLim(self,axis,lims):
        axis.trap_traj.config.accel_limit += lims
        axis.trap_traj.config.decel_limit += lims

    def addAccelLims(self,lims):
        self.addAccelLim(self.driver1.axis0,lims)
        self.addAccelLim(self.driver1.axis1,lims)
        self.addAccelLim(self.driver2.axis0,lims)
        self.addAccelLim(self.driver2.axis1,lims)
        self.addAccelLim(self.driver3.axis0,lims)
        self.addAccelLim(self.driver3.axis1,lims)
        self.addAccelLim(self.driver4.axis0,lims)
        self.addAccelLim(self.driver4.axis1,lims)

    def getCount(self,driver):
        return [driver.axis0.encoder.pos_estimate,driver.axis1.encoder.pos_estimate];

    def getCounts(self):
        return [self.getCount(self.driver1),self.getCount(self.driver2),self.getCount(self.driver3),self.getCount(self.driver4)];

    def getVel(self,driver):
        return [driver.axis0.encoder.vel_estimate,driver.axis1.encoder.vel_estimate];

    def getVels(self):
        return [self.getVel(self.driver1),self.getVel(self.driver2),self.getVel(self.driver3),self.getVel(self.driver4)];

    def getCurrent(self,driver):
        return [driver.axis0.motor.current_control.Iq_measured,driver.axis1.motor.current_control.Iq_measured]

    def getCurrents(self):
        return [self.getCurrent(self.driver1),self.getCurrent(self.driver2),self.getCurrent(self.driver3),self.getCurrent(self.driver4)];


    def getBusVoltage(self):
        return [self.driver1.vbus_voltage,self.driver2.vbus_voltage,self.driver3.vbus_voltage,self.driver4.vbus_voltage];

    def getTemp(self,driverx):
        return [self.driver1.axis0.get_temp(),self.driver1.axis1.get_temp()];

    def getTemps(self):
        return [self.getTemp(self.driver1),self.getTemp(self.driver2),self.getTemp(self.driver3),self.getTemp(self.driver4)];


    def writePosFile(self,saved):
        if(globals.dataOn == 1):
            motorPos = self.getCounts()
            timeArray.append(time.time() - globals.startTime)
            motors0.append(motorPos[0][0])
            motors1.append(motorPos[0][1])
            motors2.append(motorPos[1][0])
            motors3.append(motorPos[1][1])
            motors4.append(motorPos[2][0])
            motors5.append(motorPos[2][1])
            motors6.append(motorPos[3][0])
            motors7.append(motorPos[3][1])

        elif(globals.dataOn == 0 and saved == 0):
            np.savetxt('MotorPositions.txt',np.c_[timeArray,motors0,motors1,motors2,motors3,motors4,motors5,motors6,motors7],fmt="%.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f")
            saved = 1

    def writeVelFile(self,saved):
        if(globals.dataOn == 1):
            motorVel = self.getVels()
            timeArray.append(time.time() - globals.startTime)
            motorsVel0.append(motorVel[0][0])
            motorsVel1.append(motorVel[0][1])
            motorsVel2.append(motorVel[1][0])
            motorsVel3.append(motorVel[1][1])
            motorsVel4.append(motorVel[2][0])
            motorsVel5.append(motorVel[2][1])
            motorsVel6.append(motorVel[3][0])
            motorsVel7.append(motorVel[3][1])

        elif(globals.dataOn == 0 and saved == 0):
            np.savetxt('MotorVelocities.txt',np.c_[timeArray,motorsVel0,motorsVel1,motorsVel2,motorsVel3,motorsVel4,motorsVel5,motorsVel6,motorsVel7],fmt="%.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f")
            saved = 1

    def writeCurFile(self,saved):
        if(globals.dataOn == 1):
            motorCur = self.getCurrents()
            timeArray.append(time.time() - globals.startTime)
            motorsCur0.append(motorCur[0][0])
            motorsCur1.append(motorCur[0][1])
            motorsCur2.append(motorCur[1][0])
            motorsCur3.append(motorCur[1][1])
            motorsCur4.append(motorCur[2][0])
            motorsCur5.append(motorCur[2][1])
            motorsCur6.append(motorCur[3][0])
            motorsCur7.append(motorCur[3][1])

        elif(globals.dataOn == 0 and saved == 0):
            np.savetxt('MotorCurrents.txt',np.c_[timeArray,motorsCur0,motorsCur1,motorsCur2,motorsCur3,motorsCur4,motorsCur5,motorsCur6,motorsCur7],fmt="%.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f")
            saved = 1

    def toDeg(self,counts):
        return 360*(counts)*(1/8192)

    def toCount(self,degrees):
        return 8192*(degrees)*(1/360)

    def toLeg(self,counts):
        return counts*(1/5)

    def toMotor(self,counts):
        return counts*5

    def getAngle(self,driverx):
        es1 = driverx.axis0.encoder.pos_estimate
        es2 = driverx.axis1.encoder.pos_estimate
        return [self.toDeg(self.toLeg(es1)),self.toDeg(self.toLeg(es2))]

    def getAngles(self):
        return np.array([self.getAngle(self.driver1),self.getAngle(self.driver2),self.getAngle(self.driver3),self.getAngle(self.driver4)]);

    def getEncOffsets(self):
        ar = self.getAngles()
        ar10 = 86 + ar[0][0]
        ar11 = 86 + ar[0][1]

        ar20 = 86 + ar[1][0]
        ar21 = 86 + ar[1][1]

        ar30 = 86 - ar[2][0]
        ar31 = 86 - ar[2][1]

        ar40 = 86 - ar[3][0]
        ar41 = 86 - ar[3][1]
        
        return [ar10,ar11,ar20,ar21,ar30,ar31,ar40,ar41];

    def setPos(self,driver,pos):
        driver.axis0.controller.pos_setpoint = pos
        driver.axis1.controller.pos_setpoint = pos

    def setPosAll(self,pos):
        self.driver1.axis0.controller.pos_setpoint = pos[0]
        self.driver1.axis1.controller.pos_setpoint = pos[1]

        self.driver2.axis0.controller.pos_setpoint = pos[2]
        self.driver2.axis1.controller.pos_setpoint = pos[3]

        self.driver3.axis0.controller.pos_setpoint = pos[4]
        self.driver3.axis1.controller.pos_setpoint = pos[5]

        self.driver4.axis0.controller.pos_setpoint = pos[6]
        self.driver4.axis1.controller.pos_setpoint = pos[7]        

    def setAngles(self,rads):
        posNow = np.zeros(len(rads))
        for i in range(len(rads)):
            posNow[i] = self.toMotor(self.toCount(math.degrees(rads[i])))
        return posNow;

    def setTraj(self,driver,pos):
        driver.axis0.controller.move_to_pos(pos[0])
        driver.axis1.controller.move_to_pos(pos[1])

    def setTrajs(self,posList):
        self.setTraj(self.driver1,posList[0])
        self.setTraj(self.driver2,posList[1])
        self.setTraj(self.driver3,posList[2])
        self.setTraj(self.driver4,posList[3])

    def rotation(self,theta):
        R = np.array([[math.cos(theta),-math.sin(theta)],[math.sin(theta),math.cos(theta)]])
        return R

    def symmetric(self,alpha1,alpha2,l1,l2):
        base = [0,0]
        leftshoulder = np.array([l1*math.cos(alpha2),l1*math.sin(alpha2)])
        rightshoulder = np.array([l1*math.cos(alpha2),-l1*math.sin(alpha2)])
        foot = np.array([l1*math.cos(alpha2)+math.sqrt((l2**2)-(l1**2)*(math.sin(alpha2))**2),0])

        R = self.rotation(alpha1)
        leftshoulder = R.dot(leftshoulder)
        rightshoulder = R.dot(rightshoulder)
        foot = R.dot(foot)

        return [base,leftshoulder,rightshoulder,foot]; 

    def getPath_Walk(self):
        alpha1 = np.array([np.zeros(n),np.zeros(n),np.zeros(n),np.zeros(n)])
        alpha2 = np.array([np.zeros(n),np.zeros(n),np.zeros(n),np.zeros(n)])

        path1 = np.array([np.zeros(n),np.zeros(n),np.zeros(n),np.zeros(n)])
        path2 = np.array([np.zeros(n),np.zeros(n),np.zeros(n),np.zeros(n)])

        alpha_1, alpha_2, alpha_3, alpha_4 = np.zeros(n,dtype=float), np.zeros(n,dtype=float), np.zeros(n,dtype=float), np.zeros(n,dtype=float)       
        l_1, l_2, l_3, l_4 = np.zeros(n,dtype=float), np.zeros(n,dtype=float), np.zeros(n,dtype=float), np.zeros(n,dtype=float)

        print('Calculating equations of l and alpha')
        l1_t = ls1[0]+ls1[1]*t1+ls1[2]*np.power(t1,2)+ls1[3]*np.power(t1,3)+ls1[4]*np.power(t1,4)+ls1[5]*np.power(t1,5)+ls1[6]*np.power(t1,6)
        l2_t = ls2[0]+ls2[1]*t2+ls2[2]*np.power(t2,2)+ls2[3]*np.power(t2,3)+ls2[4]*np.power(t2,4)+ls2[5]*np.power(t2,5)+ls2[6]*np.power(t2,6)

        a1_t = as1[0]+as1[1]*t1+as1[2]*np.power(t1,2)+as1[3]*np.power(t1,3)+as1[4]*np.power(t1,4)+as1[5]*np.power(t1,5)+as1[6]*np.power(t1,6)
        a2_t = as2[0]+as2[1]*t2+as2[2]*np.power(t2,2)+as2[3]*np.power(t2,3)+as2[4]*np.power(t2,4)+as2[5]*np.power(t2,5)+as2[6]*np.power(t2,6)

        print('Calculating path of each leg!')
        print('Length is ',len(t1)+len(t2)-1)
        for i in range(len(t1)+len(t2)):
            if (i<n/4):
                alpha_1[i] = a1_t[i]
                l_1[i] = l1_t[i]
                alpha_2[i] = a2_t[i]
                l_2[i] = l2_t[i]
                alpha_3[i] = a2_t[int(i+n/4)]
                l_3[i] = l2_t[int(i+n/4)]
                alpha_4[i] = a2_t[int(i+n/2)]
                l_4[i] = l2_t[int(i+n/2)]
                
            elif (i<n/2):
                alpha_1[i] = a2_t[int(i-n/4)]
                l_1[i] = l2_t[int(i-n/4)]
                alpha_2[i] = a2_t[i]
                l_2[i] = l2_t[i]
                alpha_3[i] = a2_t[int(i+n/4)]
                l_3[i] = l2_t[int(i+n/4)]
                alpha_4[i] = a1_t[int(i-n/4)]
                l_4[i] = l1_t[int(i-n/4)]
            elif (i<3*n/4):
                alpha_1[i] = a2_t[int(i-n/4)]
                l_1[i] = l2_t[int(i-n/4)]
                alpha_2[i] = a2_t[i]
                l_2[i] = l2_t[i]
                alpha_3[i] = a1_t[int(i-n/2)]
                l_3[i] = l1_t[int(i-n/2)]
                alpha_4[i] = a2_t[int(i-n/2)]
                l_4[i] = l2_t[int(i-n/2)]
            else:
                alpha_1[i] = a2_t[int(i-n/4)]
                l_1[i] = l2_t[int(i-n/4)]
                alpha_2[i] = a1_t[int(i-3*n/4)]
                l_2[i] = l1_t[int(i-3*n/4)]
                alpha_3[i] = a2_t[int(i-3*n/4)]
                l_3[i] = l2_t[int(i-3*n/4)]
                alpha_4[i] = a2_t[int(i-n/2)]
                l_4[i] = l2_t[int(i-n/2)]
            
            alpha_s = np.array([alpha_1,alpha_2,alpha_3,alpha_4])
            l_s = np.array([l_1,l_2,l_3,l_4])
            print('Alphas ',alpha_s[0][i],' ',alpha_s[2][i])
            #print('L ',l_s)
            for j in range(4):
                alpha1[j][i] = (alpha_s[j][i] - math.pi/2)
                alpha2[j][i] = (math.acos((((globals.l1**2)+((l_s[j][i])**2)-(globals.l2**2))/(2*globals.l1*l_s[j][i]))))
         
                self.leftShoulderx[j][i] = (self.symmetric(alpha1[j][i],alpha2[j][i],globals.l1,globals.l2)[1][0]) 
                self.rightShoulderx[j][i] = (self.symmetric(alpha1[j][i],alpha2[j][i],globals.l1,globals.l2)[2][0])
                self.leftShouldery[j][i] = (self.symmetric(alpha1[j][i],alpha2[j][i],globals.l1,globals.l2)[1][1]) 
                self.rightShouldery[j][i] = (self.symmetric(alpha1[j][i],alpha2[j][i],globals.l1,globals.l2)[2][1])

                theta1[j][i] = (math.acos(self.rightShouldery[j][i]/globals.l1))
                theta2[j][i] = (math.acos(self.leftShouldery[j][i]/globals.l1))

                if(i>n-2):
                    path1[j] = (self.setAngles(theta1[j]))
                    path2[j] = (self.setAngles(theta2[j]))

            print('Iteration ',i)
        
        print('Walk Path calculated!')
        return [theta1,theta2];



    def getPath_Trot(self):
        alpha1 = np.array([np.zeros(n),np.zeros(n),np.zeros(n),np.zeros(n)])
        alpha2 = np.array([np.zeros(n),np.zeros(n),np.zeros(n),np.zeros(n)])

        path1 = np.array([np.zeros(n),np.zeros(n),np.zeros(n),np.zeros(n)])
        path2 = np.array([np.zeros(n),np.zeros(n),np.zeros(n),np.zeros(n)])

        alpha_1, alpha_2, alpha_3, alpha_4 = np.zeros(n,dtype=float), np.zeros(n,dtype=float), np.zeros(n,dtype=float), np.zeros(n,dtype=float)       
        l_1, l_2, l_3, l_4 = np.zeros(n,dtype=float), np.zeros(n,dtype=float), np.zeros(n,dtype=float), np.zeros(n,dtype=float)

        print('Calculating equations of l and alpha')
        l1_t = ls1_t[0]+ls1_t[1]*t1_trot+ls1_t[2]*np.power(t1_trot,2)+ls1_t[3]*np.power(t1_trot,3)+ls1_t[4]*np.power(t1_trot,4)+ls1_t[5]*np.power(t1_trot,5)+ls1_t[6]*np.power(t1_trot,6)
        l2_t = ls2_t[0]+ls2_t[1]*t2_trot+ls2_t[2]*np.power(t2_trot,2)+ls2_t[3]*np.power(t2_trot,3)+ls2_t[4]*np.power(t2_trot,4)+ls2_t[5]*np.power(t2_trot,5)+ls2_t[6]*np.power(t2_trot,6)

        a1_t = as1_t[0]+as1_t[1]*t1_trot+as1_t[2]*np.power(t1_trot,2)+as1_t[3]*np.power(t1_trot,3)+as1_t[4]*np.power(t1_trot,4)+as1_t[5]*np.power(t1_trot,5)+as1_t[6]*np.power(t1_trot,6)
        a2_t = as2_t[0]+as2_t[1]*t2_trot+as2_t[2]*np.power(t2_trot,2)+as2_t[3]*np.power(t2_trot,3)+as2_t[4]*np.power(t2_trot,4)+as2_t[5]*np.power(t2_trot,5)+as2_t[6]*np.power(t2_trot,6)

        l1_t2 = ls1_t2[0]+ls1_t2[1]*t1_trot+ls1_t2[2]*np.power(t1_trot,2)+ls1_t2[3]*np.power(t1_trot,3)+ls1_t2[4]*np.power(t1_trot,4)+ls1_t2[5]*np.power(t1_trot,5)+ls1_t2[6]*np.power(t1_trot,6)
        l2_t2 = ls2_t2[0]+ls2_t2[1]*t2_trot+ls2_t2[2]*np.power(t2_trot,2)+ls2_t2[3]*np.power(t2_trot,3)+ls2_t2[4]*np.power(t2_trot,4)+ls2_t2[5]*np.power(t2_trot,5)+ls2_t2[6]*np.power(t2_trot,6)

        a1_t2 = as1_t2[0]+as1_t2[1]*t1_trot+as1_t2[2]*np.power(t1_trot,2)+as1_t2[3]*np.power(t1_trot,3)+as1_t2[4]*np.power(t1_trot,4)+as1_t2[5]*np.power(t1_trot,5)+as1_t2[6]*np.power(t1_trot,6)
        a2_t2 = as2_t2[0]+as2_t2[1]*t2_trot+as2_t2[2]*np.power(t2_trot,2)+as2_t2[3]*np.power(t2_trot,3)+as2_t2[4]*np.power(t2_trot,4)+as2_t2[5]*np.power(t2_trot,5)+as2_t2[6]*np.power(t2_trot,6)

        print('Calculating path of each leg!')
        print('Length is ',len(t1_trot)+len(t2_trot)-1)
        for i in range(n):
            if (i<n/2):
                alpha_1[i] = a1_t[i]
                l_1[i] = l1_t[i]
                alpha_2[i] = a2_t2[i]
                l_2[i] = l2_t2[i]
                alpha_3[i] = a1_t2[i]
                l_3[i] = l1_t2[i]
                alpha_4[i] = a2_t[i]
                l_4[i] = l2_t[i]
                
            
            else:
                alpha_1[i] = a2_t[int(i-n/2)]
                l_1[i] = l2_t[int(i-n/2)]
                alpha_2[i] = a1_t2[int(i-n/2)]
                l_2[i] = l1_t2[int(i-n/2)]
                alpha_3[i] = a2_t2[int(i-n/2)]
                l_3[i] = l2_t2[int(i-n/2)]
                alpha_4[i] = a1_t[int(i-n/2)]
                l_4[i] = l1_t[int(i-n/2)]
            
            
            alpha_s = np.array([alpha_1,alpha_2,alpha_3,alpha_4])
            l_s = np.array([l_1,l_2,l_3,l_4])
            print('Alphas ',alpha_s[0][i],' ',alpha_s[2][i])
            #print('L ',l_s)
            for j in range(4):
                alpha1[j][i] = (alpha_s[j][i] - math.pi/2)
                alpha2[j][i] = (math.acos((((globals.l1**2)+((l_s[j][i])**2)-(globals.l2**2))/(2*globals.l1*l_s[j][i]))))
         
                self.leftShoulderx[j][i] = (self.symmetric(alpha1[j][i],alpha2[j][i],globals.l1,globals.l2)[1][0]) 
                self.rightShoulderx[j][i] = (self.symmetric(alpha1[j][i],alpha2[j][i],globals.l1,globals.l2)[2][0])
                self.leftShouldery[j][i] = (self.symmetric(alpha1[j][i],alpha2[j][i],globals.l1,globals.l2)[1][1]) 
                self.rightShouldery[j][i] = (self.symmetric(alpha1[j][i],alpha2[j][i],globals.l1,globals.l2)[2][1])

                theta1_t[j][i] = (math.acos(self.rightShouldery[j][i]/globals.l1))
                theta2_t[j][i] = (math.acos(self.leftShouldery[j][i]/globals.l1))

                if(i>n-2):
                    path1[j] = (self.setAngles(theta1[j]))
                    path2[j] = (self.setAngles(theta2[j]))

            print('Iteration ',i)
        
        print('Trot Path calculated!')
        print(theta1_t)
        print(theta2_t)
        return [theta1_t,theta2_t];




    def getPath_halfTrot(self):
        alpha1 = np.array([np.zeros(nh),np.zeros(nh),np.zeros(nh),np.zeros(nh)])
        alpha2 = np.array([np.zeros(nh),np.zeros(nh),np.zeros(nh),np.zeros(nh)])

        path1 = np.array([np.zeros(nh),np.zeros(nh),np.zeros(nh),np.zeros(nh)])
        path2 = np.array([np.zeros(nh),np.zeros(nh),np.zeros(nh),np.zeros(nh)])

        alpha_1, alpha_2, alpha_3, alpha_4 = np.zeros(nh,dtype=float), np.zeros(nh,dtype=float), np.zeros(nh,dtype=float), np.zeros(nh,dtype=float)       
        l_1, l_2, l_3, l_4 = np.zeros(nh,dtype=float), np.zeros(nh,dtype=float), np.zeros(nh,dtype=float), np.zeros(nh,dtype=float)

        print('Calculating equations of l and alpha')
        l1_t = ls1_t[0]+ls1_t[1]*t1_th+ls1_t[2]*np.power(t1_th,2)+ls1_t[3]*np.power(t1_th,3)+ls1_t[4]*np.power(t1_th,4)+ls1_t[5]*np.power(t1_th,5)+ls1_t[6]*np.power(t1_th,6)
        l2_t = ls2_t[0]+ls2_t[1]*t2_th+ls2_t[2]*np.power(t2_th,2)+ls2_t[3]*np.power(t2_th,3)+ls2_t[4]*np.power(t2_th,4)+ls2_t[5]*np.power(t2_th,5)+ls2_t[6]*np.power(t2_th,6)

        a1_t = as1_t[0]+as1_t[1]*t1_th+as1_t[2]*np.power(t1_th,2)+as1_t[3]*np.power(t1_th,3)+as1_t[4]*np.power(t1_th,4)+as1_t[5]*np.power(t1_th,5)+as1_t[6]*np.power(t1_th,6)
        a2_t = as2_t[0]+as2_t[1]*t2_th+as2_t[2]*np.power(t2_th,2)+as2_t[3]*np.power(t2_th,3)+as2_t[4]*np.power(t2_th,4)+as2_t[5]*np.power(t2_th,5)+as2_t[6]*np.power(t2_th,6)

        print('Calculating path of each leg!')
        print('Length is ',nh)
        for i in range(nh):
            if (i<nh/4):
                alpha_1[i] = a1_t[i]
                l_1[i] = l1_t[i]
                alpha_2[i] = a2_t[int(i+nh/2)]
                l_2[i] = l2_t[int(i+nh/2)]
                alpha_3[i] = a1_t[i]
                l_3[i] = l1_t[i]
                alpha_4[i] = a2_t[int(i+nh/2)]
                l_4[i] = l2_t[int(i+nh/2)]
            
            elif (i<nh/2):
                alpha_1[i] = 0
                l_1[i] = l_down[int(i-nh/4)]
                alpha_2[i] = a2_t[int(i+nh/2)]
                l_2[i] = l2_t[int(i+nh/2)]
                alpha_3[i] = 0
                l_3[i] = l_down[int(i-nh/4)]
                alpha_4[i] = a2_t[int(i+nh/2)]
                l_4[i] = l2_t[int(i+nh/2)]

            elif (i<3*nh/4):
                alpha_1[i] = a2_t[i]
                l_1[i] = l2_t[i]
                alpha_2[i] = a1_t[int(i-nh/2)]
                l_2[i] = l1_t[int(i-nh/2)]
                alpha_3[i] = a2_t[i]
                l_3[i] = l2_t[i]
                alpha_4[i] = a1_t[int(i-nh/2)]
                l_4[i] = l1_t[int(i-nh/2)]

            else:
                alpha_1[i] = a2_t[i]
                l_1[i] = l2_t[i]
                alpha_2[i] = 0
                l_2[i] = l_down[int(i-3*nh/4)]
                alpha_3[i] = a2_t[i]
                l_3[i] = l2_t[i]
                alpha_4[i] = 0
                l_4[i] = l_down[int(i-3*nh/4)]
            
            
            alpha_s = np.array([alpha_1,alpha_2,alpha_3,alpha_4])
            l_s = np.array([l_1,l_2,l_3,l_4])
            print('Alphas ',alpha_s[0][i],' ',alpha_s[2][i])
            #print('L ',l_s)
            for j in range(4):
                alpha1[j][i] = (alpha_s[j][i] - math.pi/2)
                alpha2[j][i] = (math.acos((((globals.l1**2)+((l_s[j][i])**2)-(globals.l2**2))/(2*globals.l1*l_s[j][i]))))
         
                self.leftShoulderxh[j][i] = (self.symmetric(alpha1[j][i],alpha2[j][i],globals.l1,globals.l2)[1][0]) 
                self.rightShoulderxh[j][i] = (self.symmetric(alpha1[j][i],alpha2[j][i],globals.l1,globals.l2)[2][0])
                self.leftShoulderyh[j][i] = (self.symmetric(alpha1[j][i],alpha2[j][i],globals.l1,globals.l2)[1][1]) 
                self.rightShoulderyh[j][i] = (self.symmetric(alpha1[j][i],alpha2[j][i],globals.l1,globals.l2)[2][1])

                theta1_ht[j][i] = (math.acos(self.rightShoulderyh[j][i]/globals.l1))
                theta2_ht[j][i] = (math.acos(self.leftShoulderyh[j][i]/globals.l1))

                #if(i>nh-2):
                #    path1[j] = (self.setAngles(theta1[j]))
                #    path2[j] = (self.setAngles(theta2[j]))

            print('Iteration ',i)
        
        print(theta1_ht)
        print(theta2_ht)
        print('Trot Half Path calculated!')
        return [theta1_ht,theta2_ht];

    def getPath_Bound(self):
        alpha1 = np.array([np.zeros(n),np.zeros(n),np.zeros(n),np.zeros(n)])
        alpha2 = np.array([np.zeros(n),np.zeros(n),np.zeros(n),np.zeros(n)])

        path1 = np.array([np.zeros(n),np.zeros(n),np.zeros(n),np.zeros(n)])
        path2 = np.array([np.zeros(n),np.zeros(n),np.zeros(n),np.zeros(n)])

        alpha_1, alpha_2, alpha_3, alpha_4 = np.zeros(n,dtype=float), np.zeros(n,dtype=float), np.zeros(n,dtype=float), np.zeros(n,dtype=float)       
        l_1, l_2, l_3, l_4 = np.zeros(n,dtype=float), np.zeros(n,dtype=float), np.zeros(n,dtype=float), np.zeros(n,dtype=float)

        print('Calculating equations of l and alpha')
        l1_t = ls1_t[0]+ls1_t[1]*t1_trot+ls1_t[2]*np.power(t1_trot,2)+ls1_t[3]*np.power(t1_trot,3)+ls1_t[4]*np.power(t1_trot,4)+ls1_t[5]*np.power(t1_trot,5)+ls1_t[6]*np.power(t1_trot,6)
        l2_t = ls2_t[0]+ls2_t[1]*t2_trot+ls2_t[2]*np.power(t2_trot,2)+ls2_t[3]*np.power(t2_trot,3)+ls2_t[4]*np.power(t2_trot,4)+ls2_t[5]*np.power(t2_trot,5)+ls2_t[6]*np.power(t2_trot,6)

        a1_t = as1_t[0]+as1_t[1]*t1_trot+as1_t[2]*np.power(t1_trot,2)+as1_t[3]*np.power(t1_trot,3)+as1_t[4]*np.power(t1_trot,4)+as1_t[5]*np.power(t1_trot,5)+as1_t[6]*np.power(t1_trot,6)
        a2_t = as2_t[0]+as2_t[1]*t2_trot+as2_t[2]*np.power(t2_trot,2)+as2_t[3]*np.power(t2_trot,3)+as2_t[4]*np.power(t2_trot,4)+as2_t[5]*np.power(t2_trot,5)+as2_t[6]*np.power(t2_trot,6)

        l1_t2 = ls1_t2[0]+ls1_t2[1]*t1_trot+ls1_t2[2]*np.power(t1_trot,2)+ls1_t2[3]*np.power(t1_trot,3)+ls1_t2[4]*np.power(t1_trot,4)+ls1_t2[5]*np.power(t1_trot,5)+ls1_t2[6]*np.power(t1_trot,6)
        l2_t2 = ls2_t2[0]+ls2_t2[1]*t2_trot+ls2_t2[2]*np.power(t2_trot,2)+ls2_t2[3]*np.power(t2_trot,3)+ls2_t2[4]*np.power(t2_trot,4)+ls2_t2[5]*np.power(t2_trot,5)+ls2_t2[6]*np.power(t2_trot,6)

        a1_t2 = as1_t2[0]+as1_t2[1]*t1_trot+as1_t2[2]*np.power(t1_trot,2)+as1_t2[3]*np.power(t1_trot,3)+as1_t2[4]*np.power(t1_trot,4)+as1_t2[5]*np.power(t1_trot,5)+as1_t2[6]*np.power(t1_trot,6)
        a2_t2 = as2_t2[0]+as2_t2[1]*t2_trot+as2_t2[2]*np.power(t2_trot,2)+as2_t2[3]*np.power(t2_trot,3)+as2_t2[4]*np.power(t2_trot,4)+as2_t2[5]*np.power(t2_trot,5)+as2_t2[6]*np.power(t2_trot,6)

        print('Calculating path of each leg!')
        print('Length is ',len(t1_trot)+len(t2_trot)-1)
        for i in range(n):
            if (i<n/2):
                alpha_1[i] = a1_t[i]
                l_1[i] = l1_t[i]
                alpha_2[i] = a2_t2[i]
                l_2[i] = l2_t2[i]
                alpha_3[i] = a2_t2[i]
                l_3[i] = l2_t2[i]
                alpha_4[i] = a1_t[i]
                l_4[i] = l1_t[i]
                
            
            else:
                alpha_1[i] = a2_t[int(i-n/2)]
                l_1[i] = l2_t[int(i-n/2)]
                alpha_2[i] = a1_t2[int(i-n/2)]
                l_2[i] = l1_t2[int(i-n/2)]
                alpha_3[i] = a1_t2[int(i-n/2)]
                l_3[i] = l1_t2[int(i-n/2)]
                alpha_4[i] = a2_t[int(i-n/2)]
                l_4[i] = l2_t[int(i-n/2)]
            
            
            alpha_s = np.array([alpha_1,alpha_2,alpha_3,alpha_4])
            l_s = np.array([l_1,l_2,l_3,l_4])
            print('Alphas ',alpha_s[0][i],' ',alpha_s[2][i])
            #print('L ',l_s)
            for j in range(4):
                alpha1[j][i] = (alpha_s[j][i] - math.pi/2)
                alpha2[j][i] = (math.acos((((globals.l1**2)+((l_s[j][i])**2)-(globals.l2**2))/(2*globals.l1*l_s[j][i]))))
         
                self.leftShoulderx[j][i] = (self.symmetric(alpha1[j][i],alpha2[j][i],globals.l1,globals.l2)[1][0]) 
                self.rightShoulderx[j][i] = (self.symmetric(alpha1[j][i],alpha2[j][i],globals.l1,globals.l2)[2][0])
                self.leftShouldery[j][i] = (self.symmetric(alpha1[j][i],alpha2[j][i],globals.l1,globals.l2)[1][1]) 
                self.rightShouldery[j][i] = (self.symmetric(alpha1[j][i],alpha2[j][i],globals.l1,globals.l2)[2][1])

                theta1_b[j][i] = (math.acos(self.rightShouldery[j][i]/globals.l1))
                theta2_b[j][i] = (math.acos(self.leftShouldery[j][i]/globals.l1))

            print('Iteration ',i)
        
        print('Bound Path calculated!')
        print(theta1_b)
        print(theta2_b)
        return [theta1_b,theta2_b];
