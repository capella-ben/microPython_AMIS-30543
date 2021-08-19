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

print("poistion:", myDriver.readPosition())

if myDriver.readThermalWarning():
    print("Driver too hot!")
else:
    print("Driver cool as a cucumber")


for i in range(0, 50):
    print(myDriver.moveStepsAcc(int(random() * 10000), int(random() * 1000 + 1000), 20, bool(round(random())), 2100, True))




myDriver.disableDriver()