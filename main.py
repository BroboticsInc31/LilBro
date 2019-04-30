import time as time_
import os, struct, array
from fcntl import ioctl
import robot
import math
import numpy as np
import odrive
import globals
import sys
import threading
import leds

print('Finding drivers...')

globals.initialize()
lilbro = robot.robot()
led = leds.leds()

LedThrd = threading.Thread(target = led.main)
LedThrd.daemon = True
LedThrd.start()

lilbro.findDrivers('388937803437','2061376D3548','206A3379304B','385337753437')

print('Drivers found',lilbro.getDriver1(),' ',lilbro.getDriver2(),' ',lilbro.getDriver3(),' ',lilbro.getDriver4())

alpha2 = []
legParms = []
alpha2m = []
alpha2_s = []
legParmsm = []
legParms_s = []
l_m = np.linspace(0.17,0.14,6)
l = np.linspace(0.135,0.275,50)
l_squat = np.linspace(0.14,0.17,20)

alpha = 0
l1 = 0.1
l2 = 0.22
theta1 = []
theta2 = []
theta1m = []
theta2m = []
theta1_s = []
theta2_s = []
n = 32
nh = 7

lilbro.setStates(1)
lilbro.setPGains(75)
lilbro.setVelGains(0.001057)
lilbro.setVelIntGains(0)
lilbro.setTrajAll(150000,105000,105000,0)
lilbro.setVelLims(200000)

# Joystick Code --------------------------------------

# JS state storage
axis_states = {}
button_states = {}

# These constants were borrowed from linux/input.h
axis_names = {
    0x00 : 'x',
    0x01 : 'y',
    0x02 : 'z',
    0x03 : 'rx',
    0x04 : 'ry',
    0x05 : 'rz',
    0x06 : 'trottle',
    0x07 : 'rudder',
    0x08 : 'wheel',
    0x09 : 'gas',
    0x0a : 'brake',
    0x10 : 'hat0x',
    0x11 : 'hat0y',
    0x12 : 'hat1x',
    0x13 : 'hat1y',
    0x14 : 'hat2x',
    0x15 : 'hat2y',
    0x16 : 'hat3x',
    0x17 : 'hat3y',
    0x18 : 'pressure',
    0x19 : 'distance',
    0x1a : 'tilt_x',
    0x1b : 'tilt_y',
    0x1c : 'tool_width',
    0x20 : 'volume',
    0x28 : 'misc',
}

button_names = {
    0x120 : 'trigger',
    0x121 : 'thumb',
    0x122 : 'thumb2',
    0x123 : 'top',
    0x124 : 'top2',
    0x125 : 'pinkie',
    0x126 : 'base',
    0x127 : 'base2',
    0x128 : 'base3',
    0x129 : 'base4',
    0x12a : 'base5',
    0x12b : 'base6',
    0x12f : 'dead',
    0x130 : 'a',
    0x131 : 'b',
    0x132 : 'c',
    0x133 : 'x',
    0x134 : 'y',
    0x135 : 'z',
    0x136 : 'tl',
    0x137 : 'tr',
    0x138 : 'tl2',
    0x139 : 'tr2',
    0x13a : 'select',
    0x13b : 'start',
    0x13c : 'mode',
    0x13d : 'thumbl',
    0x13e : 'thumbr',

    0x220 : 'dpad_up',
    0x221 : 'dpad_down',
    0x222 : 'dpad_left',
    0x223 : 'dpad_right',

    # XBox 360 controller uses these codes.
    0x2c0 : 'dpad_left',
    0x2c1 : 'dpad_right',
    0x2c2 : 'dpad_up',
    0x2c3 : 'dpad_down',
}

axis_map = []
button_map = []

def ctrl_map(x,in_min,in_max,out_min,out_max):
    return (x-in_min)*(out_max-out_min)/(in_max-in_min)+out_min

# Open the joystick device.
fn = '/dev/input/js0'
print('Opening %s...' % fn)

try:
    jsdev = open(fn, 'rb')

except IOError:
    print('No PS3 Controller connected')
    print('Please press the PS button to connect...')

    while True:
        if os.path.exists('/dev/input/js0'):
            print('Controller connected')

            jsdev = open(fn, 'rb')
            break

# Get the device name.

buf = bytearray(63)
# buf = array.array('u', ['\0'] * 64)
ioctl(jsdev, 0x80006a13 + (0x10000 * len(buf)), buf) # JSIOCGNAME(len)
# Get rid of random padding
buf = buf.rstrip(b'\0')
js_name = str(buf, encoding='utf-8')
print('Device name: %s' % js_name)

# Get number of axes and buttons.
buf = array.array('B', [0])
ioctl(jsdev, 0x80016a11, buf) # JSIOCGAXES
num_axes = buf[0]

buf = array.array('B', [0])
ioctl(jsdev, 0x80016a12, buf) # JSIOCGBUTTONS
num_buttons = buf[0]

# Get the axis map.
buf = array.array('B', [0] * 0x40)
ioctl(jsdev, 0x80406a32, buf) # JSIOCGAXMAP

for axis in buf[:num_axes]:
    axis_name = axis_names.get(axis, 'unknown(0x%02x)' % axis)
    axis_map.append(axis_name)
    axis_states[axis_name] = 0.0

# Get the button map.
buf = array.array('H', [0] * 200)
ioctl(jsdev, 0x80406a34, buf) # JSIOCGBTNMAP

for btn in buf[:num_buttons]:
    btn_name = button_names.get(btn, 'unknown(0x%03x)' % btn)
    button_map.append(btn_name)
    button_states[btn_name] = 0
    
def readJS():
    global armed
    global setpt_on
    global angle_cnt
    global jsdev
    global inTime
    global varTime
    global trigValue, map1, map2
    global gain
    global pos_0
    global pos_1
    global sweepOn
    global encFlag, dataFlag, dataSaved, R1Flag, lock 
    global offset, walkDir
    global paths, trotPath, thPath, squatOn
    global pathOn, manSquat, pathCalc, trotCalc, trotOn, thOn, thCalc, marchOn
    global off1, off2, off3, off4
    global standby
    standby = 1
    walkDir = 1
    pathOn = 0
    trotOn = 0
    thOn = 0
    squatOn = 0
    pathCalc = 0
    trotCalc = 0
    thCalc = 0
    map1 = len(l_m)/2
    map2 = len(l_m)/2
    marchOn = 0
    paths = []
    trotPath = []
    thPath = []
    offset = []
    encFlag = 1
    dataFlag = 1
    R1Flag = 0
    lock = 1
    dataSaved = 0
    sweepOn = 0
    gain = 20
    walkPath = 0
    pos_0 = -1000
    pos_1 = -1000
    trigValue = 0
    off1 = []
    off2 = []
    off3 = []
    off4 = []
    armed = False
    while True:

        # Read the joystick
        try:
            evbuf = jsdev.read(8)

        # If the controller disconnects during operation, turn off motors and wait for reconnect
        except IOError:
            lilbro.setState(lilbro.driver1,1)
            lilbro.setState(lilbro.driver2,1)
            lilbro.setState(lilbro.driver3,1)
            lilbro.setState(lilbro.driver4,1)

            print('No PS3 Controller connected')
            print('Please press the PS button to connect...')

            while True:
                if os.path.exists('/dev/input/js0'):
                    print('Controller connected')

                    evbuf = jsdev.read(8)
                    break

        if evbuf:
            time, value, type, number = struct.unpack('IhBB', evbuf)

            # Determine if evbuf is the initial value
            if type & 0x80:
                # print((initial)),
                continue

            # Determine if evbuf is a button
            if type & 0x01:
                button = button_map[number]
                if button:
                   button_states[button] = value
#                    # Print states (debug only)
#                    print(value)
#                   if value:
#                        print("%s pressed" % (button))
#                    else:
#                        print("%s released" % (button))

            if button_states['start'] and (armed == False):
                lilbro.setState(lilbro.driver1,8)
                lilbro.setState(lilbro.driver2,8)
                lilbro.setState(lilbro.driver3,8)
                lilbro.setState(lilbro.driver4,8)
                armed = True
                print('Motors Armed!')

            if button_states['select'] and (armed == True):
                lilbro.setState(lilbro.driver1,1)
                lilbro.setState(lilbro.driver2,1)
                lilbro.setState(lilbro.driver3,1)
                lilbro.setState(lilbro.driver4,1)
                armed = False
                sweepOn = 0
                print("Motors Unarmed!")
                
            if button_states['dpad_up'] and lock == 0:   
                lilbro.addPGain(lilbro.driver1,10)
                lilbro.addPGain(lilbro.driver4,10)
                print('Front P Gain is ',lilbro.getPGain(lilbro.driver1.axis0))

            if button_states['dpad_down'] and lock == 0:
                lilbro.addPGain(lilbro.driver1,-10)
                lilbro.addPGain(lilbro.driver4,-10)
                print('Front P Gain is ',lilbro.getPGain(lilbro.driver1.axis0))

            if button_states['dpad_right'] and lock == 0:   
                lilbro.addPGain(lilbro.driver2,10)
                lilbro.addPGain(lilbro.driver3,10)
                print('Back P Gain is ',lilbro.getPGain(lilbro.driver2.axis0))

            if button_states['dpad_left'] and lock == 0:
                lilbro.addPGain(lilbro.driver2,-10)
                lilbro.addPGain(lilbro.driver3,-10)
                print('Back P Gain is ',lilbro.getPGain(lilbro.driver2.axis0))

            if button_states['thumbr']:
                lock = 0
                print("lock OFF")

            if button_states['thumbl']:
                lock = 1
                print("lock ON")
                print("Bus voltage is ",lilbro.getBusVoltage())

            if button_states['tr']:
                if value:
                    globals.modeStat = 1
                    if button_states['dpad_up']:
                        if(globals.modeNum == 7):
                            globals.modeNum = 7
                        else:
                            globals.modeNum = globals.modeNum + 1
                            globals.decMode = globals.modeNum + 1
                    elif button_states['dpad_down']:
                        if(globals.modeNum == 3):
                            globals.modeNum = 3
                        else:
                            globals.modeNum = globals.modeNum - 1
                            globals.decMode = globals.modeNum + 1
                    
                    if button_states['a']:
                        standby = 0
                        globals.modeStat = 0


            if button_states['tl']:
                if button_states['b']:
                    if(encFlag == 1):
                        print("Encoder angle offsets measurement ready!");
                        lilbro.setStates(1) 
                        armed = 0
                                
                    elif(encFlag == -1):
                        print("Encoder angle offsets measured!")
                        offset = lilbro.getEncOffsets()
                        print("Encoder offsets are ",offset)
                                
                    encFlag = encFlag*-1

                if button_states['dpad_left']:
                    lilbro.addAccelLims(-5000)
                    print('Accel lims are ',lilbro.driver1.axis0.trap_traj.config.accel_limit)

                if button_states['dpad_right']:
                    lilbro.addAccelLims(5000)
                    print('Accel lims are ',lilbro.driver1.axis0.trap_traj.config.accel_limit)

            if button_states['b']:
                if value:
                    standby = 1
                    print('Standby On')

            if button_states['y'] and globals.modeNum == 7:
                if(walkDir == 1):
                    print("Walk Backwards!");
                elif(walkDir == -1):
                    print("Walk Forward!")
                walkDir = walkDir*-1

            if button_states['x'] and armed == 1 and standby == 1:
                if value:
                    print("Home postion")
                    lilbro.driver1.axis0.controller.move_to_pos(lilbro.toMotor(lilbro.toCount(-50+offset[0])))
                    lilbro.driver1.axis1.controller.move_to_pos(lilbro.toMotor(lilbro.toCount(-50+offset[1])))
                    lilbro.driver2.axis0.controller.move_to_pos(lilbro.toMotor(lilbro.toCount(-50+offset[2])))
                    lilbro.driver2.axis1.controller.move_to_pos(lilbro.toMotor(lilbro.toCount(-50+offset[3])))
                    lilbro.driver3.axis0.controller.move_to_pos(lilbro.toMotor(lilbro.toCount(50-1*offset[4])))
                    lilbro.driver3.axis1.controller.move_to_pos(lilbro.toMotor(lilbro.toCount(50-1*offset[5])))
                    lilbro.driver4.axis0.controller.move_to_pos(lilbro.toMotor(lilbro.toCount(50-1*offset[6])))
                    lilbro.driver4.axis1.controller.move_to_pos(lilbro.toMotor(lilbro.toCount(50-1*offset[7])))                    

            if type & 0x02:
                fvalue = value / 32767.0
                axis = axis_map[number]
                if(axis == 'ry'):
                    fvalue = (value / 32767.0) + 1
                    trigValue = fvalue
                    map1 = ctrl_map(fvalue,0,2,0,len(l_m)-1)
                    axis_states[axis] = fvalue
                    print(trigValue)
                    inTime = 0.75*fvalue

                if(axis == 'y'):
                    fvalue2 = (value / 32767.0) + 1
                    map2 = ctrl_map(fvalue2,0,2,0,len(l_m)-1)

# Start the readJS thread. Reads joystick in the "background"
readJSThrd = threading.Thread(target=readJS)
readJSThrd.daemon = True
readJSThrd.start()

for i in range(len(l)):
    alpha2.append(math.acos((((l1**2)+((l[i])**2)-(l2**2))/(2*l1*l[i]))))
    alpha1 = alpha - math.pi/2
    legParms.append(lilbro.symmetric(alpha1,alpha2[i],l1,l2))
    theta1.append(math.atan2(-legParms[i][1][0],legParms[i][1][1]))
    theta2.append(math.atan2(legParms[i][2][0],legParms[i][2][1]))

for i in range(len(l_m)):
    alpha2m.append(math.acos((((l1**2)+((l_m[i])**2)-(l2**2))/(2*l1*l_m[i]))))
    alpha1m = alpha - math.pi/2
    legParmsm.append(lilbro.symmetric(alpha1m,alpha2m[i],l1,l2))
    theta1m.append(math.atan2(-legParmsm[i][1][0],legParmsm[i][1][1]))
    theta2m.append(math.atan2(legParmsm[i][2][0],legParmsm[i][2][1]))

for k in range(len(l_squat)):
    alpha2_s.append(math.acos((((l1**2)+((l_squat[k])**2)-(l2**2))/(2*l1*l_squat[k]))))
    alpha1_s = alpha - math.pi/2
    legParms_s.append(lilbro.symmetric(alpha1_s,alpha2_s[k],l1,l2))
    theta1_s.append(math.atan2(-legParms_s[k][1][0],legParms_s[k][1][1]))
    theta2_s.append(math.atan2(legParms_s[k][2][0],legParms_s[k][2][1]))

offset = [29,16,64.5,40.5,61,14.5,78,54]

stalpha2 = math.acos((((l1**2)+((0.17**2)-(l2**2))/(2*l1*0.17))))
stlegParms = lilbro.symmetric(-math.pi/2,stalpha2,l1,l2)
sttheta1 = math.atan2(-stlegParms[1][0],stlegParms[1][1])
sttheta2 = math.atan2(stlegParms[2][0],stlegParms[2][1])

while True:
    try:

        if(standby == 1):
            globals.error = lilbro.isError()
            time_.sleep(0.0005)
        else:
            if(armed == 1 and globals.modeStat == 1):
                lilbro.driver1.axis0.controller.move_to_pos(lilbro.toMotor(lilbro.toCount(-50+offset[0])))
                lilbro.driver1.axis1.controller.move_to_pos(lilbro.toMotor(lilbro.toCount(-50+offset[1])))
                lilbro.driver2.axis0.controller.move_to_pos(lilbro.toMotor(lilbro.toCount(-50+offset[2])))
                lilbro.driver2.axis1.controller.move_to_pos(lilbro.toMotor(lilbro.toCount(-50+offset[3])))
                lilbro.driver3.axis0.controller.move_to_pos(lilbro.toMotor(lilbro.toCount(50-1*offset[4])))
                lilbro.driver3.axis1.controller.move_to_pos(lilbro.toMotor(lilbro.toCount(50-1*offset[5])))
                lilbro.driver4.axis0.controller.move_to_pos(lilbro.toMotor(lilbro.toCount(50-1*offset[6])))
                lilbro.driver4.axis1.controller.move_to_pos(lilbro.toMotor(lilbro.toCount(50-1*offset[7])))

            if((globals.modeNum == 3 and armed ==1) and globals.modeStat == 0):
                lilbro.driver1.axis0.controller.move_to_pos(lilbro.toMotor(lilbro.toCount(-50+offset[0])))
                lilbro.driver1.axis1.controller.move_to_pos(lilbro.toMotor(lilbro.toCount(-50+offset[1])))
                lilbro.driver2.axis0.controller.move_to_pos(lilbro.toMotor(lilbro.toCount(-50+offset[2])))
                lilbro.driver2.axis1.controller.move_to_pos(lilbro.toMotor(lilbro.toCount(-50+offset[3])))
                lilbro.driver3.axis0.controller.move_to_pos(lilbro.toMotor(lilbro.toCount(50-1*offset[4])))
                lilbro.driver3.axis1.controller.move_to_pos(lilbro.toMotor(lilbro.toCount(50-1*offset[5])))
                lilbro.driver4.axis0.controller.move_to_pos(lilbro.toMotor(lilbro.toCount(50-1*offset[6])))
                lilbro.driver4.axis1.controller.move_to_pos(lilbro.toMotor(lilbro.toCount(50-1*offset[7])))


            if(globals.modeNum == 4 and armed == 1 and globals.modeStat == 0):
                
                for i in range(len(l_squat)):
                    if(globals.modeNum != 4 or armed == 0 or globals.modeStat == 1 or standby == 1):
                        break

                    posNow1 = lilbro.toMotor(lilbro.toCount(math.degrees(theta1_s[i])+offset[0]))
                    posNow2 = lilbro.toMotor(lilbro.toCount(math.degrees(theta2_s[i])+offset[1]))
                    posNow3 = lilbro.toMotor(lilbro.toCount(math.degrees(theta2_s[i])+offset[2]))
                    posNow4 = lilbro.toMotor(lilbro.toCount(math.degrees(theta1_s[i])+offset[3]))
                    posNow5 = lilbro.toMotor(lilbro.toCount(-math.degrees(theta2_s[i])-offset[4]))
                    posNow6 = lilbro.toMotor(lilbro.toCount(-math.degrees(theta1_s[i])-offset[5]))
                    posNow7 = lilbro.toMotor(lilbro.toCount(-math.degrees(theta1_s[i])-offset[6]))
                    posNow8 = lilbro.toMotor(lilbro.toCount(-math.degrees(theta2_s[i])-offset[7]))

                    lilbro.driver1.axis0.controller.move_to_pos(posNow1)
                    lilbro.driver1.axis1.controller.move_to_pos(posNow2)

                    lilbro.driver2.axis0.controller.move_to_pos(posNow3)
                    lilbro.driver2.axis1.controller.move_to_pos(posNow4)

                    lilbro.driver3.axis0.controller.move_to_pos(posNow5)
                    lilbro.driver3.axis1.controller.move_to_pos(posNow6)

                    lilbro.driver4.axis0.controller.move_to_pos(posNow7)
                    lilbro.driver4.axis1.controller.move_to_pos(posNow8)

                    print('Theta1,Theta2 =  ',lilbro.getAngle(lilbro.driver1))
            
                    time_.sleep(0.02)

                theta1f_s = np.flipud(theta1_s)
                theta2f_s = np.flipud(theta2_s)
                for j in range(len(l_squat)):
                    if(globals.modeNum != 4 or armed == 0 or globals.modeStat == 1 or standby == 1):
                        break
                            
                    posNow1 = lilbro.toMotor(lilbro.toCount(math.degrees(theta1f_s[j])+offset[0]))
                    posNow2 = lilbro.toMotor(lilbro.toCount(math.degrees(theta2f_s[j])+offset[1]))
                    posNow3 = lilbro.toMotor(lilbro.toCount(math.degrees(theta2f_s[j])+offset[2]))
                    posNow4 = lilbro.toMotor(lilbro.toCount(math.degrees(theta1f_s[j])+offset[3]))
                    posNow5 = lilbro.toMotor(lilbro.toCount(-math.degrees(theta2f_s[j])-offset[4]))
                    posNow6 = lilbro.toMotor(lilbro.toCount(-math.degrees(theta1f_s[j])-offset[5]))
                    posNow7 = lilbro.toMotor(lilbro.toCount(-math.degrees(theta1f_s[j])-offset[6]))
                    posNow8 = lilbro.toMotor(lilbro.toCount(-math.degrees(theta2f_s[j])-offset[7]))
                    lilbro.driver1.axis0.controller.move_to_pos(posNow1)
                    lilbro.driver1.axis1.controller.move_to_pos(posNow2)

                    lilbro.driver2.axis0.controller.move_to_pos(posNow3)
                    lilbro.driver2.axis1.controller.move_to_pos(posNow4)

                    lilbro.driver3.axis0.controller.move_to_pos(posNow5)
                    lilbro.driver3.axis1.controller.move_to_pos(posNow6)

                    lilbro.driver4.axis0.controller.move_to_pos(posNow7)
                    lilbro.driver4.axis1.controller.move_to_pos(posNow8)   

                    print('Theta1,Theta2 =  ',lilbro.getAngle(lilbro.driver1))
                    time_.sleep(0.02)

            if(globals.modeNum == 5 and armed == 1 and globals.modeStat == 0):
                
                for i in range(len(l)):
                    if(globals.modeNum != 5 or armed == 0 or globals.modeStat == 1 or standby == 1):
                        break

                    posNow1 = lilbro.toMotor(lilbro.toCount(math.degrees(theta1[i])+offset[0]))
                    posNow2 = lilbro.toMotor(lilbro.toCount(math.degrees(theta2[i])+offset[1]))
                    posNow3 = lilbro.toMotor(lilbro.toCount(math.degrees(theta2[i])+offset[2]))
                    posNow4 = lilbro.toMotor(lilbro.toCount(math.degrees(theta1[i])+offset[3]))
                    posNow5 = lilbro.toMotor(lilbro.toCount(-math.degrees(theta2[i])-offset[4]))
                    posNow6 = lilbro.toMotor(lilbro.toCount(-math.degrees(theta1[i])-offset[5]))
                    posNow7 = lilbro.toMotor(lilbro.toCount(-math.degrees(theta1[i])-offset[6]))
                    posNow8 = lilbro.toMotor(lilbro.toCount(-math.degrees(theta2[i])-offset[7]))

                    lilbro.driver1.axis0.controller.move_to_pos(posNow1)
                    lilbro.driver1.axis1.controller.move_to_pos(posNow2)

                    lilbro.driver2.axis0.controller.move_to_pos(posNow3)
                    lilbro.driver2.axis1.controller.move_to_pos(posNow4)

                    lilbro.driver3.axis0.controller.move_to_pos(posNow5)
                    lilbro.driver3.axis1.controller.move_to_pos(posNow6)

                    lilbro.driver4.axis0.controller.move_to_pos(posNow7)
                    lilbro.driver4.axis1.controller.move_to_pos(posNow8)

                    print('Theta1,Theta2 =  ',lilbro.getAngle(lilbro.driver1))
            
                    time_.sleep(0.015)

                standTime = time_.time()
                while(time_.time()-standTime <= 8):
                    if(globals.modeNum != 5 or armed == 0 or globals.modeStat == 1 or standby == 1):
                        break
                    time_.sleep(0.1)

                theta1f = np.flipud(theta1)
                theta2f = np.flipud(theta2)
                for j in range(len(l)):
                    if(globals.modeNum != 5 or armed == 0 or globals.modeStat == 1 or standby == 1):
                        break
                            
                    posNow1 = lilbro.toMotor(lilbro.toCount(math.degrees(theta1f[j])+offset[0]))
                    posNow2 = lilbro.toMotor(lilbro.toCount(math.degrees(theta2f[j])+offset[1]))
                    posNow3 = lilbro.toMotor(lilbro.toCount(math.degrees(theta2f[j])+offset[2]))
                    posNow4 = lilbro.toMotor(lilbro.toCount(math.degrees(theta1f[j])+offset[3]))
                    posNow5 = lilbro.toMotor(lilbro.toCount(-math.degrees(theta2f[j])-offset[4]))
                    posNow6 = lilbro.toMotor(lilbro.toCount(-math.degrees(theta1f[j])-offset[5]))
                    posNow7 = lilbro.toMotor(lilbro.toCount(-math.degrees(theta1f[j])-offset[6]))
                    posNow8 = lilbro.toMotor(lilbro.toCount(-math.degrees(theta2f[j])-offset[7]))
                    lilbro.driver1.axis0.controller.move_to_pos(posNow1)
                    lilbro.driver1.axis1.controller.move_to_pos(posNow2)

                    lilbro.driver2.axis0.controller.move_to_pos(posNow3)
                    lilbro.driver2.axis1.controller.move_to_pos(posNow4)

                    lilbro.driver3.axis0.controller.move_to_pos(posNow5)
                    lilbro.driver3.axis1.controller.move_to_pos(posNow6)

                    lilbro.driver4.axis0.controller.move_to_pos(posNow7)
                    lilbro.driver4.axis1.controller.move_to_pos(posNow8)   

                    print('Theta1,Theta2 =  ',lilbro.getAngle(lilbro.driver1))
                    time_.sleep(0.015)

                squatTime = time_.time()
                while(time_.time()-squatTime <= 3):
                    if(globals.modeNum != 5 or armed == 0 or globals.modeStat == 1 or standby == 1):
                        break
                    time_.sleep(0.1)

            if(globals.modeNum == 6 and armed == 1 and globals.modeStat == 0):

                for i in range(len(l_m)):
                    if(globals.modeNum != 6 or armed == 0 or globals.modeStat == 1 or standby == 1):
                        break

                    posNow1 = lilbro.toMotor(lilbro.toCount(math.degrees(theta1m[0])+offset[0]))
                    posNow2 = lilbro.toMotor(lilbro.toCount(math.degrees(theta2m[0])+offset[1]))
                    posNow3 = lilbro.toMotor(lilbro.toCount(math.degrees(theta2m[i])+offset[2]))
                    posNow4 = lilbro.toMotor(lilbro.toCount(math.degrees(theta1m[i])+offset[3]))
                    posNow5 = lilbro.toMotor(lilbro.toCount(-math.degrees(theta2m[0])-offset[4]))
                    posNow6 = lilbro.toMotor(lilbro.toCount(-math.degrees(theta1m[0])-offset[5]))
                    posNow7 = lilbro.toMotor(lilbro.toCount(-math.degrees(theta1m[i])-offset[6]))
                    posNow8 = lilbro.toMotor(lilbro.toCount(-math.degrees(theta2m[i])-offset[7]))

                    lilbro.driver1.axis0.controller.move_to_pos(posNow1)
                    lilbro.driver1.axis1.controller.move_to_pos(posNow2)

                    lilbro.driver2.axis0.controller.move_to_pos(posNow3)
                    lilbro.driver2.axis1.controller.move_to_pos(posNow4)

                    lilbro.driver3.axis0.controller.move_to_pos(posNow5)
                    lilbro.driver3.axis1.controller.move_to_pos(posNow6)

                    lilbro.driver4.axis0.controller.move_to_pos(posNow7)
                    lilbro.driver4.axis1.controller.move_to_pos(posNow8)

                    print('Theta1,Theta2 =  ',lilbro.getAngle(lilbro.driver1))
                    time_.sleep(0.005)

                theta1fm = np.flipud(theta1m)
                theta2fm = np.flipud(theta2m)
                for j in range(len(l_m)):
                    if(globals.modeNum != 6 or armed == 0 or globals.modeStat == 1 or standby == 1):
                        break
                            
                    posNow1 = lilbro.toMotor(lilbro.toCount(math.degrees(theta1m[0])+offset[0]))
                    posNow2 = lilbro.toMotor(lilbro.toCount(math.degrees(theta2m[0])+offset[1]))
                    posNow3 = lilbro.toMotor(lilbro.toCount(math.degrees(theta2fm[j])+offset[2]))
                    posNow4 = lilbro.toMotor(lilbro.toCount(math.degrees(theta1fm[j])+offset[3]))
                    posNow5 = lilbro.toMotor(lilbro.toCount(-math.degrees(theta2m[0])-offset[4]))
                    posNow6 = lilbro.toMotor(lilbro.toCount(-math.degrees(theta1m[0])-offset[5]))
                    posNow7 = lilbro.toMotor(lilbro.toCount(-math.degrees(theta1fm[j])-offset[6]))
                    posNow8 = lilbro.toMotor(lilbro.toCount(-math.degrees(theta2fm[j])-offset[7]))
                    lilbro.driver1.axis0.controller.move_to_pos(posNow1)
                    lilbro.driver1.axis1.controller.move_to_pos(posNow2)

                    lilbro.driver2.axis0.controller.move_to_pos(posNow3)
                    lilbro.driver2.axis1.controller.move_to_pos(posNow4)

                    lilbro.driver3.axis0.controller.move_to_pos(posNow5)
                    lilbro.driver3.axis1.controller.move_to_pos(posNow6)

                    lilbro.driver4.axis0.controller.move_to_pos(posNow7)
                    lilbro.driver4.axis1.controller.move_to_pos(posNow8)   

                    print('Theta1,Theta2 =  ',lilbro.getAngle(lilbro.driver1))
                    time_.sleep(0.005)

                for i in range(len(l_m)):
                    if(globals.modeNum != 6 or armed == 0 or globals.modeStat == 1 or standby == 1):
                        break

                    posNow1 = lilbro.toMotor(lilbro.toCount(math.degrees(theta1m[i])+offset[0]))
                    posNow2 = lilbro.toMotor(lilbro.toCount(math.degrees(theta2m[i])+offset[1]))
                    posNow3 = lilbro.toMotor(lilbro.toCount(math.degrees(theta2m[0])+offset[2]))
                    posNow4 = lilbro.toMotor(lilbro.toCount(math.degrees(theta1m[0])+offset[3]))
                    posNow5 = lilbro.toMotor(lilbro.toCount(-math.degrees(theta2m[i])-offset[4]))
                    posNow6 = lilbro.toMotor(lilbro.toCount(-math.degrees(theta1m[i])-offset[5]))
                    posNow7 = lilbro.toMotor(lilbro.toCount(-math.degrees(theta1m[0])-offset[6]))
                    posNow8 = lilbro.toMotor(lilbro.toCount(-math.degrees(theta2m[0])-offset[7]))

                    lilbro.driver1.axis0.controller.move_to_pos(posNow1)
                    lilbro.driver1.axis1.controller.move_to_pos(posNow2)

                    lilbro.driver2.axis0.controller.move_to_pos(posNow3)
                    lilbro.driver2.axis1.controller.move_to_pos(posNow4)

                    lilbro.driver3.axis0.controller.move_to_pos(posNow5)
                    lilbro.driver3.axis1.controller.move_to_pos(posNow6)

                    lilbro.driver4.axis0.controller.move_to_pos(posNow7)
                    lilbro.driver4.axis1.controller.move_to_pos(posNow8)

                    print('Theta1,Theta2 =  ',lilbro.getAngle(lilbro.driver1))
                    time_.sleep(0.005)

                for j in range(len(l_m)):
                    if(globals.modeNum != 6 or armed == 0 or globals.modeStat == 1 or standby == 1):
                        break
                            
                    posNow1 = lilbro.toMotor(lilbro.toCount(math.degrees(theta1fm[j])+offset[0]))
                    posNow2 = lilbro.toMotor(lilbro.toCount(math.degrees(theta2fm[j])+offset[1]))
                    posNow3 = lilbro.toMotor(lilbro.toCount(math.degrees(theta2m[0])+offset[2]))
                    posNow4 = lilbro.toMotor(lilbro.toCount(math.degrees(theta1m[0])+offset[3]))
                    posNow5 = lilbro.toMotor(lilbro.toCount(-math.degrees(theta2fm[j])-offset[4]))
                    posNow6 = lilbro.toMotor(lilbro.toCount(-math.degrees(theta1fm[j])-offset[5]))
                    posNow7 = lilbro.toMotor(lilbro.toCount(-math.degrees(theta1m[0])-offset[6]))
                    posNow8 = lilbro.toMotor(lilbro.toCount(-math.degrees(theta2m[0])-offset[7]))
                    lilbro.driver1.axis0.controller.move_to_pos(posNow1)
                    lilbro.driver1.axis1.controller.move_to_pos(posNow2)

                    lilbro.driver2.axis0.controller.move_to_pos(posNow3)
                    lilbro.driver2.axis1.controller.move_to_pos(posNow4)

                    lilbro.driver3.axis0.controller.move_to_pos(posNow5)
                    lilbro.driver3.axis1.controller.move_to_pos(posNow6)

                    lilbro.driver4.axis0.controller.move_to_pos(posNow7)
                    lilbro.driver4.axis1.controller.move_to_pos(posNow8)   

                    print('Theta1,Theta2 =  ',lilbro.getAngle(lilbro.driver1))
                    time_.sleep(0.005)




            if(globals.modeNum == 7 and armed == 1 and globals.modeStat == 0):
                if(trotCalc == 0):
                    trotPath = lilbro.getPath_Trot()
                    trotCalc = 1

                    trotPathFlip1 = np.flipud(trotPath[1][0])
                    trotPathFlip2 = np.flipud(trotPath[0][0])
                    trotPathFlip3 = np.flipud(trotPath[0][1])
                    trotPathFlip4 = np.flipud(trotPath[1][1])
                    trotPathFlip5 = np.flipud(trotPath[0][2])
                    trotPathFlip6 = np.flipud(trotPath[1][2])
                    trotPathFlip7 = np.flipud(trotPath[1][3])
                    trotPathFlip8 = np.flipud(trotPath[0][3])
            
                if(walkDir == 1):
                    for i in range(n):
                         if(globals.modeNum != 7 or armed == 0 or globals.modeStat == 1 or standby == 1):
                             break

                         posNow1 = lilbro.toMotor(lilbro.toCount(-math.degrees(trotPath[1][0][i])+offset[0]))
                         posNow2 = lilbro.toMotor(lilbro.toCount(-math.degrees(trotPath[0][0][i])+offset[1]))

                         posNow3 = lilbro.toMotor(lilbro.toCount(-math.degrees(trotPath[0][1][i])+offset[2]))
                         posNow4 = lilbro.toMotor(lilbro.toCount(-math.degrees(trotPath[1][1][i])+offset[3]))
                       
                         posNow5 = lilbro.toMotor(lilbro.toCount(math.degrees(trotPath[0][2][i])-offset[4]))
                         posNow6 = lilbro.toMotor(lilbro.toCount(math.degrees(trotPath[1][2][i])-offset[5]))

                         posNow7 = lilbro.toMotor(lilbro.toCount(math.degrees(trotPath[1][3][i])-offset[6]))
                         posNow8 = lilbro.toMotor(lilbro.toCount(math.degrees(trotPath[0][3][i])-offset[7]))

                         lilbro.driver1.axis0.controller.move_to_pos(posNow1)
                         lilbro.driver1.axis1.controller.move_to_pos(posNow2)

                         lilbro.driver2.axis0.controller.move_to_pos(posNow3)
                         lilbro.driver2.axis1.controller.move_to_pos(posNow4)

                         lilbro.driver3.axis0.controller.move_to_pos(posNow5)
                         lilbro.driver3.axis1.controller.move_to_pos(posNow6)

                         lilbro.driver4.axis0.controller.move_to_pos(posNow7)
                         lilbro.driver4.axis1.controller.move_to_pos(posNow8)

                         #print('Theta1,Theta2 =  ',lilbro.getAngle(lilbro.driver1))
                         if(lock == 0):
                             print(lilbro.getCurrents())

                         time_.sleep(0.0005)

                if(walkDir == -1):
                    for i in range(n):
                         if(globals.modeNum != 7 or armed == 0 or globals.modeStat == 1 or standby == 1):
                             break

                         posNow1 = lilbro.toMotor(lilbro.toCount(-math.degrees(trotPathFlip1[i])+offset[0]))
                         posNow2 = lilbro.toMotor(lilbro.toCount(-math.degrees(trotPathFlip2[i])+offset[1]))

                         posNow3 = lilbro.toMotor(lilbro.toCount(-math.degrees(trotPathFlip3[i])+offset[2]))
                         posNow4 = lilbro.toMotor(lilbro.toCount(-math.degrees(trotPathFlip4[i])+offset[3]))
                       
                         posNow5 = lilbro.toMotor(lilbro.toCount(math.degrees(trotPathFlip5[i])-offset[4]))
                         posNow6 = lilbro.toMotor(lilbro.toCount(math.degrees(trotPathFlip6[i])-offset[5]))

                         posNow7 = lilbro.toMotor(lilbro.toCount(math.degrees(trotPathFlip7[i])-offset[6]))
                         posNow8 = lilbro.toMotor(lilbro.toCount(math.degrees(trotPathFlip8[i])-offset[7]))

                         lilbro.driver1.axis0.controller.move_to_pos(posNow1)
                         lilbro.driver1.axis1.controller.move_to_pos(posNow2)

                         lilbro.driver2.axis0.controller.move_to_pos(posNow3)
                         lilbro.driver2.axis1.controller.move_to_pos(posNow4)

                         lilbro.driver3.axis0.controller.move_to_pos(posNow5)
                         lilbro.driver3.axis1.controller.move_to_pos(posNow6)

                         lilbro.driver4.axis0.controller.move_to_pos(posNow7)
                         lilbro.driver4.axis1.controller.move_to_pos(posNow8)

                         #print('Theta1,Theta2 =  ',lilbro.getAngle(lilbro.driver1))
                         if(lock == 0):
                             print(lilbro.getCurrents())

                         time_.sleep(0.0005)


    except (KeyboardInterrupt):
        lilbro.setStates(1) 
        sys.exit()        
