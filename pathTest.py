import robot
import math
import numpy as np
import time
import odrive

driver = odrive.find_any()

lilbro = robot.robot()
alpha2 = []

legParms = []
l = np.linspace(0.1,0.14,100)
alpha = 0
l1 = 0.1
l2 = 0.2
i = 0

theta1 = []
theta2 = []

lilbro.setState(driver,8)

input("Press enter to go to base position")

lilbro.setPos(driver,9800)

input("Press enter to continue")

for i in range(len(l)):
   # print(l[i])
    
    alpha2.append(math.acos((((l1**2)+((l[i])**2)-(l2**2))/(2*l1*l[i]))))
    alpha1 = alpha - math.pi/2

    legParms.append(lilbro.symmetric(alpha1,alpha2[i],l1,l2))
    print(legParms[i][1]," , ",legParms[i][2])

    theta1.append(math.atan2(legParms[i][1][0],legParms[i][1][1]))
    theta2.append(math.atan2(legParms[i][2][0],legParms[i][2][1]))

    print('Angles are ', math.degrees(theta1[i]),' and ', math.degrees(theta2[i]))

    posNow = lilbro.toMotor(lilbro.toCount(math.degrees(theta2[i])))
    lilbro.setPos(driver,posNow)

    time.sleep(0.05)

lilbro.setState(driver,1)
