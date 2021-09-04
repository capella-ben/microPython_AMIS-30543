# Wiring:
# Pico      -    AMIS-30543
# SPIO-RX   -    DO     (4.7K pullup)
# SPIO-TX   -    DI
# SPIO-CSK  -    CLK
# 5         -    CS
# 15        -    NXT
# Ground    -    GND
# 
# Also connect the motor power and the stepper driver.


from machine import Pin
from random import random
import time
from AMIS30543 import AMIS30543


stepPin = 15
csPin = 5



myDriver = AMIS30543(csPin, stepPin)
myDriver.resetSettings()
myDriver.setCurrentMilliamps(1000)
myDriver.setStepMode(myDriver.MicroStep8)
myDriver.accelPercent = 0.1
myDriver.enableDriver()



limit = 100 * myDriver.microStepMultiple * 50
curPos = 0

def moveShoulder(targetPos, speed=1000, acceleration = 25, current=2000):
    global curPos
    global limit
    direction = True

    print("current position before:", curPos)

    if targetPos >= limit:
        targetPos = limit
    elif targetPos<= 0:
        targetPos  = 0

    if targetPos > curPos:
        moveDif = targetPos - curPos
        direction = True
        print("moveDif:", moveDif, "\tdirection", direction)
        myDriver.moveStepsAcc(moveDif, speed, acceleration, direction, current)       
    else:
        moveDif = curPos - targetPos
        direction = False
        print("moveDif:", moveDif, "\tdirection", direction)
        myDriver.moveStepsAcc(moveDif, speed, acceleration, direction, current)       

    curPos = targetPos

"""    
moveShoulder(10 * myDriver.microStepMultiple * 50, speed=1000, acceleration=50)    
moveShoulder(20 * myDriver.microStepMultiple * 50, speed=2000, acceleration=50)    
moveShoulder(30 * myDriver.microStepMultiple * 50, speed=300, acceleration=50)    
moveShoulder(33 * myDriver.microStepMultiple * 50, speed=50, acceleration=50)    

moveShoulder(30 * myDriver.microStepMultiple * 50, speed=2000, acceleration=50)    
moveShoulder(31 * myDriver.microStepMultiple * 50, speed=2000, acceleration=50)    
moveShoulder(30 * myDriver.microStepMultiple * 50, speed=2000, acceleration=50)    
moveShoulder(32 * myDriver.microStepMultiple * 50, speed=2000, acceleration=50)    
moveShoulder(31 * myDriver.microStepMultiple * 50, speed=2000, acceleration=50)    
moveShoulder(33 * myDriver.microStepMultiple * 50, speed=2000, acceleration=50)    
moveShoulder(32 * myDriver.microStepMultiple * 50, speed=2000, acceleration=50)    
moveShoulder(34 * myDriver.microStepMultiple * 50, speed=2000, acceleration=50)    

moveShoulder(10 * myDriver.microStepMultiple * 50, speed=1500, acceleration=50)    
moveShoulder(70 * myDriver.microStepMultiple * 50, speed=2000, acceleration=30)    
moveShoulder(0  * myDriver.microStepMultiple * 50, speed=1800, acceleration=20)    
"""

for i in range(0, 10):
    moveShoulder((int(random() * 100))  * myDriver.microStepMultiple * 50, speed=int(random() * 2000 + 1000), acceleration=50, current=1500)

moveShoulder(0  * myDriver.microStepMultiple * 50, speed=3000, acceleration=20, current=1500)    



#myDriver.moveStepsAcc(50 * myDriver.microStepMultiple * 50, 1000, 25, False, 2000)
#myDriver.moveStepsAcc(100 * myDriver.microStepMultiple * 50, 2000, 20, False, 2100)
#myDriver.moveStepsAcc(50 * myDriver.microStepMultiple * 50, 1100, 50, True, 2100)

#for i in range(0, 50):
#    print(myDriver.moveStepsAcc(int(random() * 10000), int(random() * 1500 + 1000), 20, bool(round(random())), 2100, True))




myDriver.disableDriver()