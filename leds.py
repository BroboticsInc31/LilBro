import board
import neopixel
import time 
import random 

global error

class leds():
    
    def __init__(self):
        print("initialized led class")
        self.pixels = neopixel.NeoPixel(board.D18,30)

    def ledStartUpSequence(self):

        while(1):

            for i in range(5):

                self.pixels[i] = (random.randint(1,101),random.randint(1,101),random.randint(1,101))
                time.sleep(.2)


            for i in range(5):

                self.pixels[i] = (0,0,0)
                time.sleep(.2)
    
    def status(self):
        if(error == 0):
            self.pixels[1] = (0,100,0)
        else:
            self.pixels[1] = (100,0,0)
