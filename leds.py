import board
import neopixel
import time 
import random 
import numpy as np
import globals

global process 
global process1



class leds():
    
    def __init__(self):
        #print("initialized leds")
        self.pixels = neopixel.NeoPixel(board.D18,24)

    def main(self):
        process = 1
        process1 = 1
        process2 = 1
        process3 = 1
        process4 = 1
        
        for i in range(2):
            self.startUp()
        self.off()
        self.startSequence()
        self.off()
        self.init()
        time.sleep(1)
        self.off()
        self.status()
        self.static()
        self.mode()
        while True:


            if(globals.modeStat == 0 and process == 1):
                self.status()
                self.mode()
                process = 0
            elif(globals.modeStat == 0 and globals.modeNum == 3):
                self.fade()
                print('Mode 1: Standing')
            elif(globals.modeStat == 0 and globals.modeNum == 4 and process1 == 1):
                self.squats()
                process1 = 0
                print('Mode 2: Squats On')
            elif(globals.modeStat == 0 and globals.modeNum == 5 and process2 == 1):
                self.manlySquats()
                process2 = 0
                print('Mode 3: High Standing (Manly) Squats On')
            elif(globals.modeStat == 0 and globals.modeNum == 6 and process3 == 1):
                self.march()
                process3 = 0
                print('Mode 4: Marching On')
            elif(globals.modeStat == 0 and globals.modeNum == 7 and process4 == 1):
                self.walking()
                process4 = 0
                print('Mode 5: Walking On')
            elif(globals.modeStat == 1):
                self.status()
                self.mode()
                process = 1
                process1 = 1
                process2 = 1
                process3 = 1
                process4 = 1
            else:
                time.sleep(0.55)
                 



    def startUp(self):
        for i in range(12):
            self.pixels[i+12] = (35,4,0)
            self.pixels[11 - i] = (0,0,35) 
            time.sleep(0.05)
        for i in range(12):
            self.pixels[23-i] = (0,0,35)
            self.pixels[i] = (35,4,0) 
            time.sleep(0.05)


    def startSequence(self):

        for i in range(24):
            self.pixels[i] = (50,0,0)
            time.sleep(1.625)

    def init(self):
        if(globals.error == 0):
            self.pixels.fill((0,25,0))
        else:
            self.pixels.fill((25,0,0))

    def mode(self):

        if globals.modeStat == 1:
            self.pixels[1] = (25,25,25)
            time.sleep(.1)
            self.pixels[1] = (0,0,0) 
            time.sleep(.1)
        else:
            self.pixels[1] = (25,25,25)


        self.pixels[globals.modeNum] = (0,0,65)
        self.pixels[globals.decMode] = (0,0,0)

   
    def off(self):
        self.pixels.fill((0,0,0))

    def status(self):
        if(globals.error == 0):
            self.pixels[0] = (0,25,0)
        else:
            self.pixels[0] = (25,0,0)

    def static(self):
        for i in range(15):
            self.pixels[9+i]=(3,10,65)

    def fade(self):
        for i in range(15):
            self.pixels[9+i]=(3,3,3)
        for n in range(12):
            if(globals.modeStat == 1):
                break
            self.pixels[9] = (3*n,3*n,3*n)           
            self.pixels[10] = (3*n,3*n,3*n)
            self.pixels[11] = (3*n,3*n,3*n)
            self.pixels[12] = (3*n,3*n,3*n)
            self.pixels[13] = (3*n,3*n,3*n)            
            self.pixels[14] = (3*n,3*n,3*n)
            self.pixels[15] = (3*n,3*n,3*n)
            self.pixels[16] = (3*n,3*n,3*n)
            self.pixels[17] = (3*n,3*n,3*n)           
            self.pixels[18] = (3*n,3*n,3*n)
            self.pixels[19] = (3*n,3*n,3*n)
            self.pixels[20] = (3*n,3*n,3*n)
            self.pixels[21] = (3*n,3*n,3*n)            
            self.pixels[22] = (3*n,3*n,3*n)
            self.pixels[23] = (3*n,3*n,3*n)
            time.sleep(0.0005)

        for n in range(12):
            if(globals.modeStat == 1):
                break            
            self.pixels[9] = (37-3*n,37-3*n,37-3*n)
            self.pixels[10] = (37-3*n,37-3*n,37-3*n)
            self.pixels[11] = (37-3*n,37-3*n,37-3*n)
            self.pixels[12] = (37-3*n,37-3*n,37-3*n)
            self.pixels[13] = (37-3*n,37-3*n,37-3*n)
            self.pixels[14] = (37-3*n,37-3*n,37-3*n)
            self.pixels[15] = (37-3*n,37-3*n,37-3*n)
            self.pixels[16] = (37-3*n,37-3*n,37-3*n) 
            self.pixels[17] = (37-3*n,37-3*n,37-3*n)
            self.pixels[18] = (37-3*n,37-3*n,37-3*n)
            self.pixels[19] = (37-3*n,37-3*n,37-3*n)
            self.pixels[20] = (37-3*n,37-3*n,37-3*n)
            self.pixels[21] = (37-3*n,37-3*n,37-3*n)
            self.pixels[22] = (37-3*n,37-3*n,37-3*n)
            self.pixels[23] = (37-3*n,37-3*n,37-3*n)            
            time.sleep(0.0005)

    def manlySquats(self):
        for i in range(15):
            self.pixels[9+i]=(30,0,0)

    def squats(self):
        for i in range(15):
            self.pixels[9+i]=(10,30,0)       

    def march(self):
        self.pixels[9] = (25,0,0)
        self.pixels[10] = (25,0,0)
        self.pixels[11] = (30,30,0)
        self.pixels[12] = (30,30,0) 
        self.pixels[13] = (0,25,0)
        self.pixels[14] = (0,25,0)
        self.pixels[15] = (25,0,0)
        self.pixels[16] = (25,0,0)         
        self.pixels[17] = (30,30,0)
        self.pixels[18] = (30,30,0)
        self.pixels[19] = (0,25,0)
        self.pixels[20] = (0,25,0) 
        self.pixels[21] = (25,0,0)
        self.pixels[22] = (25,0,0)
        self.pixels[23] = (30,30,0)

    def walking(self):    
        self.pixels[9] = (25,0,0)
        self.pixels[10] = (25,0,0)
        self.pixels[11] = (25,0,0)
        self.pixels[12] = (25,0,0) 
        self.pixels[13] = (0,25,0)
        self.pixels[14] = (0,25,0)
        self.pixels[15] = (0,25,0)
        self.pixels[16] = (0,25,0)         
        self.pixels[17] = (0,0,25)
        self.pixels[18] = (0,0,25)
        self.pixels[19] = (0,0,25)
        self.pixels[20] = (25,0,0) 
        self.pixels[21] = (25,0,0)
        self.pixels[22] = (25,0,0)
        self.pixels[23] = (25,0,0)
       
       
       
    def miniGame(self):
        while True:

            global previous
            self.pixels[globals.idxNum] = (0,0,255)


            if globals.idxNum != previous :
                previous = globals.idxNum
                self.pixels.fill((0,0,0))
