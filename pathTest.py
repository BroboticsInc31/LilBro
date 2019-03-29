import robot
import math
import numpy as np
import time

lilbro = robot.robot()
alpha2 = []
legParms = []
l = np.linspace(0.14,0.22,50)
alpha = 0
l1 = 0.1
l2 = 0.22
i = 0

print(len(l))

for i in range(len(l)):
    print(l[i])
    alpha2.append(math.acos((((l1**2)+((l[i])**2)-(l2**2))/(2*l1*l[i]))))
    alpha1 = alpha - math.pi/2

    legParms.append(lilbro.symmetric(alpha1,alpha2[i],l1,l2))
    print("Foot positions ",legParms[i])
