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
import time, utime
from AMIS30543 import AMIS30543
import _thread


stepPin = 15
csPin = 5


def myThread():
    # Note: you cannot use time.sleep
    baton.acquire()
    myDriver.enableDriver()
    myDriver.moveStepsAcc(200 * 4 * 1, 1000, 0, 1000)
    myDriver.disableDriver()
    utime.sleep_ms(1000)
    baton.release()
    print("Baton Released")




print("Startup")


myDriver = AMIS30543(csPin, stepPin)
myDriver.resetSettings()
myDriver.setCurrentMilliamps(myDriver.idleCurrent)
myDriver.setStepMode(myDriver.MicroStep4)


baton = _thread.allocate_lock()



_thread.start_new_thread(myThread, ())

internal_led = Pin(25, Pin.OUT)

while True:
    if not baton.locked():
        _thread.start_new_thread(myThread, ())
    internal_led.toggle()
    time.sleep_ms(200)



