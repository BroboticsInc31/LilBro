import time as time_
import os, struct, array
from fcntl import ioctl
import robot
import math
import numpy as np
import time
import odrive
import globals
import sys
import threading

print('Finding drivers...')

globals.initialize()
lilbro = robot.robot()

lilbro.findDrivers(206237793548,388937803437)
print('Drivers found!')

alpha2 = []
legParms = []
l = np.linspace(0.105,0.145,100)
alpha = 0
l1 = 0.1
l2 = 0.2
theta1 = []
theta2 = []
lilbro.setStates(1)
lilbro.setGains(20)

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
    global trigValue
    global gain
    global pos_0
    global pos_1
    global sweepOn
    global encFlag
    encFlag = 0
    sweepOn = 0
    gain = 20
    walkPath = 0
    pos_0 = -1000
    pos_1 = -1000
    trigValue = 0
    armed = False
    while True:

        # Read the joystick
        try:
            evbuf = jsdev.read(8)

        # If the controller disconnects during operation, turn off motors and wait for reconnect
        except IOError:
            lilbro.setStates(1)

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
                lilbro.setStates(8)
                armed = True
                print('Motors Armed!')

            if button_states['select'] and (armed == True):
                lilbro.setStates(1)
                armed = False
                sweepOn = 0
                print("Motors Unarmed!")
                
            if button_states['dpad_up']:   
                lilbro.addPGain(10)
                print(lilbro.getPGain())

            if button_states['dpad_down']:
                lilbro.addPGain(-10)
                print(lilbro.getPGain())

            if button_states['a']:
                sweepOn = 1

            if button_states['tr']:
                if(encFlag == 1):
                    print("Encoder angle offsets measurement ready!");
                    lilbro.setStates(1)
                elif(encFlag == -1):
                    print("Encoder angle offsets measured!")
                    encOffsets = lilbro.getEncOffsets()
                    print("Encoder offsets are ",encOffsets)
                encFlag = encFlag*-1
            
            if button_states['b']:
                sweepOn = 0

            if type & 0x02:
                fvalue = value / 32767.0
                axis = axis_map[number]
                if(axis == 'ry'):
                    fvalue = (value / 32767.0) + 1
                    trigValue = fvalue           
                    axis_states[axis] = fvalue
                    print(trigValue)
                    inTime = 0.75*fvalue

# Start the readJS thread. Reads joystick in the "background"
readJSThrd = threading.Thread(target=readJS)
readJSThrd.daemon = True
readJSThrd.start()

# while(lilbro.driver1.axis0.current_state != 8):
#     if(globals.reqState == 8):
#         lilbro.setStates(globals.reqState)
#         print("RAINBOW NIII")
#     print("Mode is ",driver1.axis0.current_state," ,",driver1.axis1.current_state)
#     time.sleep(0.1)

# lilbro.setPos(driver,9800)
print("Enter start")
while(lilbro.driver1.axis0.current_state != 8):
    print('Not in closed loop')
  #  if(globals.armed == 1):
   #     lilbro.setStates(8)
    time.sleep(0.5)


print('Closed loop engaged')
while True:
    try:
        print('In loop')

        if(sweepOn == 1 and armed == 1):
           # lilbro.setGains(lilbro.getGains(lilbro.driver1)+gain)
            
            for i in range(len(l)):
                if(sweepOn == 0 or armed == 0):
                    break

                alpha2.append(math.acos((((l1**2)+((l[i])**2)-(l2**2))/(2*l1*l[i]))))
                alpha1 = alpha - math.pi/2

                legParms.append(lilbro.symmetric(alpha1,alpha2[i],l1,l2))
               # print(legParms[i][1]," , ",legParms[i][2])

                theta1.append(math.atan2(legParms[i][1][0],legParms[i][1][1]))
                theta2.append(math.atan2(legParms[i][2][0],legParms[i][2][1]))

               # print('Angles are ', math.degrees(theta1[i]),' and ', math.degrees(theta2[i]))

                posNow = lilbro.toMotor(lilbro.toCount(math.degrees(theta1[i])))
                posNow1 = lilbro.toMotor(lilbro.toCount(math.degrees(theta1[i])-18))
                lilbro.setPos(lilbro.driver1,posNow1)
                lilbro.setPos(lilbro.driver2,posNow)
                print('theta1 ',lilbro.getAngle(lilbro.driver1)[1])
                time.sleep(0.01)

            theta1f = np.flipud(theta1)
            for j in range(len(l)):
                if(sweepOn == 0 or armed == 0):
                    break
                        
                posNow = lilbro.toMotor(lilbro.toCount(math.degrees(theta1f[j])))
                posNow1 = lilbro.toMotor(lilbro.toCount(math.degrees(theta1f[j])-18))
                lilbro.setPos(lilbro.driver1,posNow1)
                lilbro.setPos(lilbro.driver2,posNow)
                print('theta1 flipped ',lilbro.getAngle(lilbro.driver1)[1])
                time.sleep(0.01)

        time.sleep(0.1)

    except (KeyboardInterrupt):
        lilbro.setStates(1)
        sys.exit()        
