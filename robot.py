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

        self.legParms = np.array([])

        global timeArray
        timeArray = []
        global motorPos
        motorPos = []
        global motor1
        motor1 = []

        global offset1, offset2, offset3, offset4
        offset1 = [18.7,21.5]
        offset2 = [35,15]
        offset3 = [31.5,49.3]
        offset4 = [20.5,45.5]

        global ls1, ls2, as1, as2, t1, t2, theta1, theta2
        ls1 = np.array([0,0,0,-320,4800,-24000,40000])
        ls2 = np.array([0.1319,0.5267,-3.8189,13.4431,-23.8683,20.5761,-6.8587])
        as1 = np.array([1,0,0,-1309,9817,-19635,0])
        as2 = np.array([-1.1312,10.3427,-64.6418,177.765,-202.0057,80.8023,0])

        t1 = np.linspace(0,0.2,20)
        t2 = np.linspace(0.2,0.8,60)

        theta1 = np.array([])
        theta2 = np.array([])

    
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
        self.setGain(self.driver1,newGain)
        self.setGain(self.driver2,newGain)
        self.setGain(self.driver3,newGain)
        self.setGain(self.driver4,newGain)

    def getPGain(self):
        return self.driver1.axis0.controller.config.pos_gain;

    def addPGain(self,incGain):
        self.driver1.axis0.controller.config.pos_gain += incGain
        self.driver1.axis1.controller.config.pos_gain += incGain

        self.driver2.axis0.controller.config.pos_gain += incGain
        self.driver2.axis1.controller.config.pos_gain += incGain

        self.driver3.axis0.controller.config.pos_gain += incGain
        self.driver3.axis1.controller.config.pos_gain += incGain

        self.driver4.axis0.controller.config.pos_gain += incGain
        self.driver4.axis1.controller.config.pos_gain += incGain

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


    def getCount(self,driver):
        return [self.driver.axis0.encoder.pos_estimate,self.driver.axis1.encoder.pos_estimate];

    def getCounts(self):
        return [self.getCount(self.driver1),self.getCount(self.driver2),self.getCount(self.driver3),self.getCount(self.driver4)];


    def getCurrent(self,driver):
        return [self.driver.axis0.motor.current_control.Iq_measured,self.driver.axis1.motor.current_control.Iq_measured]

    def getCurrents(self):
        return [self.getCurrent(self.driver1),self.getCurrent(self.driver2),self.getCurrent(self.driver3),self.getCurrent(self.driver4)];


    def getBusVoltage(self):
        return [self.driver1.vbus_voltage,self.driver2.vbus_voltage,self.driver3.vbus_voltage,self.driver4.vbus_voltage];

    def getTemp(self,driverx):
        return [self.driver1.axis0.get_temp(),self.driver1.axis1.get_temp()];

    def getTemps(self):
        return [self.getTemp(self.driver1),self.getTemp(self.driver2),self.getTemp(self.driver3),self.getTemp(self.driver4)];


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

    def getAngle(self,driverx):
        es1 = driverx.axis0.encoder.pos_estimate
        es2 = driverx.axis1.encoder.pos_estimate
        return [self.toDeg(self.toLeg(es1)),self.toDeg(self.toLeg(es2))]

    def getAngles(self):
        return np.array([self.getAngle(self.driver1),self.getAngle(self.driver2),self.getAngle(self.driver3),self.getAngle(self.driver4)]);

    def getEncOffsets(self):
        ar = self.getAngles()
        ar10 = 86 - ar[0][0]
        ar11 = 86 - ar[0][1]

        ar20 = 86 - ar[1][0]
        ar21 = 86 - ar[1][1]

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
        posNow = []
        for i in range(len(rads)):
            posNow[i] = self.toMotor(self.toCount(math.degrees(rads[i])))
        return posNow;


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

    def getPath(self,l,alpha,offset):
        alpha1 = np.array([])
        alpha2 = np.array([])

        path1 = np.array([])
        path2 = np.array([])

        alpha_1, alpha_2, alpha_3, alpha_4 = np.zeros(80)
        l_1, l_2, l_3, l_4 = np.zeros(80)

        alpha_s = np.array([alpha_1,alpha_2,alpha_3,alpha_4])
        l_s = np.array([l_1,l_2,l_3,l_4])

        print('Calculating equations of l and alpha')
        l1_t = ls1[0]+ls1[1]*t1+ls1[2]*np.power(t1,2)+ls1[3]*np.power(t1,3)+ls1[4]*np.power(t1,4)+ls1[5]*np.power(t1,5)+ls1[6]*np.power(t1,6)
        l2_t = ls2[0]+ls2[1]*t2+ls2[2]*np.power(t2,2)+ls2[3]*np.power(t2,3)+ls2[4]*np.power(t2,4)+ls2[5]*np.power(t2,5)+ls2[6]*np.power(t2,6)

        a1_t = as1[0]+as1[1]*t1+as1[2]*np.power(t1,2)+as1[3]*np.power(t1,3)+as1[4]*np.power(t1,4)+as1[5]*np.power(t1,5)+as1[6]*np.power(t1,6)
        a2_t = as2[0]+as2[1]*t2+as2[2]*np.power(t2,2)+as2[3]*np.power(t2,3)+as2[4]*np.power(t2,4)+as2[5]*np.power(t2,5)+as2[6]*np.power(t2,6)

        print('Calculating path of each leg!')

        for i in range(len(t1)+len(t2)-1):
            if (i<20):
                alpha_1[i] = a1_t[i]
                l_1[i] = l1_t[i]
                alpha_2[i] = a2_t[i]
                l_2[i] = l2_t[i]
                alpha_3[i] = a2_t[i+20]
                l_3[i] = l2_t[i+20]
                alpha_4[i] = a2_t[i+40]
                l_4[i] = l2_t[i+40]
            elif (i<40):
                alpha_1[i] = a2_t[i-20]
                l_1[i] = l2_t[i-20]
                alpha_2[i] = a2_t[i]
                l_2[i] = l2_t[i]
                alpha_3[i] = a2_t[i+20]
                l_3[i] = l2_t[i+20]
                alpha_4[i] = a1_t[i-20]
                l_4[i] = l1_t[i-20]
            elif (i<60):
                alpha_1[i] = a2_t[i-20]
                l_1[i] = l2_t[i-20]
                alpha_2[i] = a2_t[i]
                l_2[i] = l2_t[i]
                alpha_3[i] = a1_t[i-40]
                l_3[i] = l1_t[i-40]
                alpha_4[i] = a2_t[i-40]
                l_4[i] = l2_t[i-40]
            else:
                alpha_1[i] = a2_t[i-20]
                l_1[i] = l2_t[i-20]
                alpha_2[i] = a1_t[i-60]
                l_2[i] = l1_t[i-60]
                alpha_3[i] = a2_t[i-60]
                l_3[i] = l2_t[i-60]
                alpha_4[i] = a1_t[i-40]
                l_4[i] = l1_t[i-40]

            for j in range(3):
                alpha1[j].append(alpha_s[j][i] - math.pi/2)
                alpha2[j].append(math.acos((((globals.l1**2)+((l_s[j][i])**2)-(globals.l2**2))/(2*globals.l1*l_s[j][i]))))
         
                self.legParms[j].append(self.symmetric(alpha1[j][i],alpha2[j][i],globals.l1,globals.l2))

                theta1[j].append(math.atan2(self.legParms[j][i][1][0],self.legParms[j][i][1][1]))
                theta2[j].append(math.atan2(self.legParms[j][i][2][0],self.legParms[j][i][2][1]))

                if(i>78):
                    path1[j].append(self.setAngles(theta1[j]))
                    path2[j].append(self.setAngles(theta2[j]))
        
        print('Paths calculated!')
        return [path1,path2];
