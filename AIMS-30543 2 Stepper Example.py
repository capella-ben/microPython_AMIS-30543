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


stepPin1 = 15
csPin1 = 5

stepPin2 = 14
csPin2 = 13


def secondMove(drv, steps, speed, dir, current):
    """
    Function for moving a stepper in a seperate thread.
    """
    # Note: you cannot use time.sleep, use utime.sleep instead
    # Note: you cannot do SPI based commands on more than one driver at the same time. 
    baton.acquire()
    drv.moveStepsAcc(steps, speed, dir, current)
    baton.release()

def dualMove(d1, d2, st1, st2, sp1, sp2, di1, di2, c1, c2):
    """
    Move 2 steppers at the same time

    Parameters
    ----------
    d1 : AMIS30543
        AMIS30543 Driver
    d2 : AMIS30543
        AMIS30543 Driver
    st1 : int
        Steps for motor 1
    st2 : int
        Steps for motor 2
    sp1 : int
        Speed for motor 1
    sp2 : int
        Speed for motor 2
    di1 : boolean
        direction of motor 1
    di2 : boolean
        direction of motor 2
    c1 : int
        Current for motor 1
    c2 : int
        Current for motor 2
    """
    if not baton.locked():
        _thread.start_new_thread(secondMove, (d1, st1, sp1, di1, c1))
    d2.moveStepsAcc(st2, sp2, di2, c2)

    # make sure that the other thread has finished before moving on.
    while baton.locked():
        time.sleep_us(1)
    time.sleep_ms(1)   # need a little settle time before looping.



print("Startup")


myDriver1 = AMIS30543(csPin1, stepPin1)
myDriver2 = AMIS30543(csPin2, stepPin2)

myDriver1.resetSettings()
myDriver1.setCurrentMilliamps(myDriver1.idleCurrent)
myDriver1.setStepMode(myDriver1.MicroStep4)
myDriver1.enableDriver()

myDriver2.resetSettings()
myDriver2.setCurrentMilliamps(myDriver1.idleCurrent)
myDriver2.setStepMode(myDriver1.MicroStep4)
myDriver2.enableDriver()


baton = _thread.allocate_lock()

# move 2 steppers at the same time. 
dualMove(myDriver1, myDriver2, 200*4*3, 200*4*1, 2500, 300, 1, 0, 2000, 500)



myDriver1.disableDriver()
myDriver2.disableDriver()

