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
import time
from AMIS30543 import AMIS30543


stepPin = 15
csPin = 5



myDriver = AMIS30543(csPin, stepPin)
myDriver.resetSettings()
myDriver.setCurrentMilliamps(myDriver.idleCurrent)
myDriver.setStepMode(myDriver.MicroStep4)
myDriver.enableDriver()

print("poistion:", myDriver.readPosition())

if myDriver.readThermalWarning():
    print("Driver too hot!")
else:
    print("Driver cool as a cucumber")


for i in range(10):
    print("Iteration:", i)
    myDriver.moveStepsAcc(200 * 4 * 1, 1000, 0, 1000)
    myDriver.moveStepsAcc(200 * 4 * 5, 6000, 1, 1000)
    myDriver.moveStepsAcc(200 * 4 * 2, 2000, 0, 1000)
    myDriver.moveStepsAcc(200, 5000, 1, 800)
    myDriver.moveStepsAcc(300, 5000, 0, 800)
    myDriver.moveStepsAcc(400, 5000, 1, 800)
    myDriver.moveStepsAcc(500, 5000, 0, 800)
    time.sleep_ms(200)
    print()

myDriver.disableDriver()