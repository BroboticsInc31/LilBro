import robot
import math
import numpy as np
import time
import odrive
import control
import globals
import sys

globals.initialize()
lilbro = robot.robot()
ctrl = control.control()

lilbro.findDrivers(206237793548,388937803437)

alpha2 = []
legParms = []
l = np.linspace(0.1,0.14,100)
alpha = 0
l1 = 0.1
l2 = 0.2
theta1 = []
theta2 = []
lilbro.setGains(20)

readJSThrd = threading.Thread(target=ctrl.ctrl)
readJSThrd.daemon = True
readJSThrd.start()

# while(lilbro.driver1.axis0.current_state != 8):
#     if(globals.reqState == 8):
#         lilbro.setStates(globals.reqState)
#         print("RAINBOW NIII")
#     print("Mode is ",driver1.axis0.current_state," ,",driver1.axis1.current_state)
#     time.sleep(0.1)

# lilbro.setPos(driver,9800)

while True:
    try:
        
        if(globals.sweepOn == 1 and globals.reqState == 8):

            lilbro.setGains(lilbro.getGains()+globals.gainInc)
            for i in range(len(l)):
                if(globals.sweepOn == 0 or globals.reqState == 1):
                        break

                alpha2.append(math.acos((((l1**2)+((l[i])**2)-(l2**2))/(2*l1*l[i]))))
                alpha1 = alpha - math.pi/2

                legParms.append(lilbro.symmetric(alpha1,alpha2[i],l1,l2))
                print(legParms[i][1]," , ",legParms[i][2])

                theta1.append(math.atan2(legParms[i][1][0],legParms[i][1][1]))
                theta2.append(math.atan2(legParms[i][2][0],legParms[i][2][1]))

                print('Angles are ', math.degrees(theta1[i]),' and ', math.degrees(theta2[i]))

                posNow = lilbro.toMotor(lilbro.toCount(math.degrees(theta2[i])))
                lilbro.setPos(lilbro.driver1,posNow)
                lilbro.setPos(lilbro.driver2,posNow)

                time.sleep(0.05)

            theta2f = np.flipud(theta2)
            for j in range(len(l)):
                if(globals.sweepOn == 0 or globals.reqState == 1):
                        break
                        
                posNow = lilbro.toMotor(lilbro.toCount(math.degrees(theta2f[j])))
                lilbro.setPos(lilbro.driver1,posNow)
                lilbro.setPos(lilbro.driver2,posNow)

                time.sleep(0.05)

    except (KeyboardInterrupt):
        lilbro.setStates(1)
        sys.exit()        
