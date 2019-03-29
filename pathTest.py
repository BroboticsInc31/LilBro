import robot
import math
import numpy as np
import time

lilbro = robot.robot()
alpha2 = []
l = np.linspace(0.14,0.22)
alpha = 0
l1 = 0.1
l2 = 0.22

for i in range(len(l)):
    alpha2[i] = math.acos(((l1**2+l[i]**2-l2**2)/(2*l1*l[i])))
    alpha1 = alpha - math.pi/2

    legParms = lilbro.symmetric(alpha1,alpha2,l1,l2)
    print("Foot positions ",legParms[4][1]," and ",legParms[4][2])
