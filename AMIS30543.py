import time
from machine import SPI, Pin




class AMIS30543:
    """
    A class for the use of the AMIS-30543 Stepper Driver

    Attributes
    ----------
    accelPercent : float
        The percentage of the speed that is used for acceleration and decelleration.  Expressed as a float.
    idleCurrent : int
        The current in mA to use as the holding current (i.e. when the stepper is not moving)
    """

    accelPercent = 0.01         # percentage of the SPEED that is used for acceleration/decelleration
    idleCurrent = 300 # mA

    # AMIS-⁠30543 register addresses constants
    WR  = 0x0
    CR0 = 0x1
    CR1 = 0x2
    CR2 = 0x3
    CR3 = 0x9
    SR0 = 0x4
    SR1 = 0x5
    SR2 = 0x6
    SR3 = 0x7
    SR4 = 0xA

    # register variables
    wr = 0x0
    cr0 = 0x0
    cr1 = 0x0
    cr2 = 0x0
    cr3 = 0x0

    # Step Modes
    MicroStep128 = 128
    MicroStep64 = 64
    MicroStep32 = 32
    MicroStep16 = 16
    MicroStep8 = 8
    MicroStep4 = 4
    MicroStep2 = 2
    MicroStep1 = 1
    CompensatedHalf = MicroStep2
    CompensatedFullTwoPhaseOn = MicroStep1
    CompensatedFullOnePhaseOn = 200
    UncompensatedHalf = 201
    UncompensatedFull = 202

    def __init__(self, csPin, stepPin):
        """
        Parameters
        ----------
        csPin : int
            The pin number for Chip Select (AKA Slave Select)
        stepPin : int
            The pin number for NXT
        """
        self.step = Pin(stepPin, Pin.OUT)
        self.step.low()
        self.cs = Pin(csPin, Pin.OUT)
        self.cs.high()        # disble Chip Select
        self.driver = SPI(0)
        self.driver.init()
        time.sleep_ms(10)   # give the driver some time to come online

    def writeReg(self, address, value):
        """
        Private function to write a value to a register
        """
        self.cs.low() 
        #print("Address:", 0x80 | (address & 0b11111), "Value:", value)
        self.driver.write(bytes([0x80 | (address & 0b11111)]))
        self.driver.write(bytes([value]))
        self.cs.high() 

    def readReg(self, address):
        """
        Private function to read a register
        """
        self.cs.low() 
        self.driver.write(bytes([address & 0b11111]))
        myData = self.driver.read(1)
        self.cs.high() 
        return int.from_bytes(myData, 'big')

    
    def resetSettings(self):
        """
        Reset all registers to 0
        """
        self.wr = 0x0
        self.cr0 = 0x0
        self.cr1 = 0x0
        self.cr2 = 0x0
        self.cr3 = 0x0
        self.applySettings()

    def applySettings(self):
        """
        Private function to write to all registers for all saved register variables
        """
        self.writeReg(self.CR2, self.cr2)       # apparently CR2 needs to be written first.
        self.writeReg(self.WR, self.wr)
        self.writeReg(self.CR0, self.cr0)
        self.writeReg(self.CR3, self.cr3)

    def setCurrentMilliamps(self, current):
        """
        Sets the current limit in the driver

        Parameters
        ----------
        current : int
            The current in mA
        """
        # This comes from Table 13 of the AMIS-30543 datasheet.
        code = 0
        if (current >= 3000):
            code = 0b11001
        elif (current >= 2845):
            code = 0b11000
        elif (current >= 2700): 
            code = 0b10111
        elif (current >= 2440):
            code = 0b10110
        elif (current >= 2240):
            code = 0b10101
        elif (current >= 2070):
            code = 0b10100
        elif (current >= 1850):
            code = 0b10011
        elif (current >= 1695):
            code = 0b10010
        elif (current >= 1520):
            code = 0b10001
        elif (current >= 1405):
            code = 0b10000
        elif (current >= 1260):
            code = 0b01111
        elif (current >= 1150):
            code = 0b01110
        elif (current >= 1060):
            code = 0b01101
        elif (current >=  955):
            code = 0b01100
        elif (current >=  870):
            code = 0b01011
        elif (current >=  780):
            code = 0b01010
        elif (current >=  715):
            code = 0b01001
        elif (current >=  640):
            code = 0b01000
        elif (current >=  585):
            code = 0b00111
        elif (current >=  540):
            code = 0b00110
        elif (current >=  485):
            code = 0b00101
        elif (current >=  445):
            code = 0b00100
        elif (current >=  395):
            code = 0b00011
        elif (current >=  355):
            code = 0b00010
        elif (current >=  245):
            code = 0b00001

        self.cr0 = (self.cr0 & 0b11100000) | code
        self.writeReg(self.CR0, self.cr0)
    
    def setStepMode(self, mode):
        """
        Sets the stepping mode

        Parameters
        ----------
        mode : int
            Use the Microstepping constants in this class
        """
        # 1/32 micro-stepping is the default mode.
        esm = 0b000
        sm = 0b000

        if mode == self.MicroStep32:
            sm = 0b000
        elif mode == self.MicroStep16:
            sm = 0b001
        elif mode == self.MicroStep8:
            sm = 0b010
        elif mode == self.MicroStep4:
            sm = 0b011
        elif mode == self.CompensatedHalf:      # AKA MicroStep2
            sm = 0b100
        elif mode == self.UncompensatedHalf:
            sm = 0b101
        elif mode == self.UncompensatedFull:
            sm = 0b110
        elif mode == self.MicroStep128:
            esm = 0b001
        elif mode == self.MicroStep64:
            esm = 0b010
        elif mode == self.CompensatedFullTwoPhaseOn:      # AKA Microstep1
            esm = 0b011
        elif mode == self.CompensatedFullOnePhaseOn:
            esm = 0b100

        self.cr0 = (self.cr0 & ~0b11100000) | (sm << 5)
        self.cr3 = (self.cr3 & ~0b111) | esm
        self.writeReg(self.CR0, self.cr0)
        self.writeReg(self.CR3, self.cr3)
        
    def enableDriver(self):
        """
        Enables the driver.  Holding power will be applied to the stepper motor
        """
        self.cr2 = self.cr2 | 0b10000000
        self.applySettings()

    def disableDriver(self):
        """
        Disables the driver.  No power is sent to the stepper motor
        """
        self.cr2 = self.cr2 & ~0b10000000
        self.applySettings()

    def readStatusReg(self, address):
        """
        Private function to read the value of a register
        """
        # Mask off the parity bit.
        return self.readReg(address) & 0x7F
    
    def readPosition(self):
        """
        Reads the position the driver has recorded. See table 9 of the AMIS-50543 datasheet

        Returns
        -------
        int
            A number between 0 and 511
        """
        self.sr3 = self.readStatusReg(self.SR3)
        self.sr4 = self.readStatusReg(self.SR4)
        return (self.sr3 << 2) | (self.sr4 & 3)
    
    def setDirection(self, value):
        """
        Sets the relative direction via the SPI interface.  The absolute direction depends on the wiring of the stepper motor to the driver board.

        Parameters
        ----------
        value : boolean
        """
        if (value):
            self.cr1 |= 0x80
        else:
            self.cr1 &= ~0x80
        self.writeReg(self.CR1, self.cr1)

    def getDirection(self):
        """
        Get the direction setting from the driver

        Returns
        -------
        boolean
        """
        return self.cr1 >> 7 & 1

    def sleep(self):
        """
        Sleep the driver
        """
        self.cr2 |= (1 << 6)
        self.applySettings()

    def sleepStop(self):
        """
        Wake up the driver
        """
        self.cr2 &= ~(1 << 6)
        self.applySettings()

    def stepOnRisingEdge(self):
        """
        Sets the value of the NXTP configuration bit to 0, which means that new steps are triggered by a rising edge on the NXT/STEP pin.  This is the default behavior.
        """
        self.cr1 = self.cr1 &  ~0b01000000
        self.writeReg(self.CR1, self.cr1)

    def stepOnFallingEdge(self):
        """
        Sets the value of the NXTP configuration bit to 1, which means that new steps are triggered by a falling edge on the NXT/STEP pin.
        """
        self.cr1 = self.cr1 | 0b01000000
        self.writeReg(self.CR1, self.cr1)

    def setPwmFrequencyDouble(self):
        """
        Sets the PWMF bit to 1, which doubles the PWM frequency (45.6 kHz)
        """
        self.cr1 = self.cr1 | (1 << 3)
        self.writeReg(self.CR1, self.cr1)

    def setPwmFrequencyDefault(self):
        """
        Clears the PWMF bit, which sets the PWM frequency to its default value (22.8 kHz).
        """
        self.cr1 = self.cr1 & ~(1 << 3)
        self.writeReg(self.CR1, self.cr1)

    def setPwmJitterOn(self):
        """
        Sets the PWMJ bit, which enables artificial jittering in the PWM signal used to control the current to each coil.
        """
        self.cr1 = self.cr1 | (1 << 2)
        self.writeReg(self.CR1, self.cr1)

    def setPwmJitterOff(self):
        """
        Clears the PWMJ bit, which disables artificial jittering in the PWM signal used to control the current to each coil.  This is the default setting.
        """
        self.cr1 = self.cr1 & ~(1 << 2)
        self.writeReg(self.CR1, self.cr1)

    def setPwmSlope(self, emc):
        """
        This sets the EMC[1:0] bits, which determine how long it takes the PWM signal to rise or fall.  Valid values are 0 through 3.  Higher values correspond to longer rise and fall times.
        """
        self.cr1 = (self.cr1 & ~0b11) | (emc & 0b11)
        self.writeReg(self.CR1, self.cr1)

    def setSlaGainDefault(self):
        """
        Clears the SLAG bit, which configures the signal on SLA pin to have a gain of 0.5 (the default).
        """
        self.cr2 = self.cr2 & ~(1 << 5)
        self.applySettings()

    def setSlaGainHalf(self):
        """
        Sets the SLAG bit to 1, which configures the signal on SLA pin to have a gain of 0.25 (half of the default).
        """
        self.cr2 = self.cr2 | (1 << 5)
        self.applySettings()

    def setSlaTransparencyOff(self):
        """
        Set the SLAT bit to 0 (the default), which disables transparency on the SLA pin.  See the AMIS-30543 datasheet for more information.
        """
        self.cr2 = self.cr2 & ~(1 << 4)
        self.applySettings()

    def setSlaTransparencyOn(self):
        """
        Sets the SLAT bit to 1, which enables transparency on the SLA pin. See the AMIS-30543 datasheet for more information.
        """
        self.cr2 = self.cr2 | (1 << 4)
        self.applySettings()

    def readNonLatchedStatusFlags(self):
        """
        Reads the status flags from registers SR0.  These flags are not latched, which means they will be cleared as soon as the condition causing them is no longer detected.  See the AMIS-30543 datasheet for more information.

        Returns
        ------
        int
            The raw value of SR0
        """
        return self.readStatusReg(self.SR0)

    def readLatchedStatusFlagsAndClear(self):
        """
        Reads the latched status flags from registers SR1 and SR2.  They are cleared as a side effect.

        WARNING: Calling this function clears the latched error bits in SR1 and SR2, which might allow the motor driver outputs to reactivate.  The AMIS-30543 datasheet says "successive reading the SPI Status Registers 1 and 2 in case of a short circuit condition, may lead to damage to the drivers".

        Returns
        -------
        int
            The return value is a 16-bit unsigned integer that has one bit for each status flag.
        """
        self.sr1 = self.readStatusReg(self.SR1)
        self.sr2 = self.readStatusReg(self.SR2)
        return (self.sr2 << 8) | self.sr1

    def readThermalWarning(self):
        """
        Is the driver in thermal warning

        Returns
        -------
        boolean
        """
        self.sr0 = self.readStatusReg(self.SR0)
        return (self.sr0 & 0b01000000) >> 6


    def moveStepsAcc(self, steps, speed, direction, current, debug=False):
        """
        Moves the stepper motor with an acceleration and decelleration profile. 
        Will fail if the driver is in thermal warning

        Parameters
        ----------
        steps : int
            The total number of steps to move
        speed : int
            The speed in Hz to move
        direction : boolean
        current : int
            The current in mA to apply to the motor when moving.  After the move the motor is returned to the idleCurrent
        debug : boolean
            If True will print debug output

        Returns
        -------
        boolean
            Indicates success or failure (depending on thermal warning)
        """
        if not self.readThermalWarning():
            if debug:
                print("Move Steps:", steps, " @ speed:", speed, " in dierction:", direction, " using", current, "mA")
            self.setDirection(direction)
            self.setCurrentMilliamps(current)
            loopTimeUs = round(((1/speed) * 1000000) - 5)

            # calculate the number of steps for Acceleration, Deceleration and full speed.
            aSteps = self.accelPercent * speed
            dSteps = self.accelPercent * speed
            # Calculation Acceleration slope
            aM = (1/aSteps) /2      
            dM = (1/dSteps) /2
            # remaining steps at full speed.
            mSteps = steps - (aSteps + dSteps)

            # Accelerate
            for i in range(0, aSteps):
                self.step.high()
                time.sleep_us(5)
                self.step.low()
                multiplier = 1.5 - aM * i
                time.sleep_us(round(loopTimeUs * multiplier))
            
            # Full Speed
            for i in range(1, mSteps):
                self.step.high()
                time.sleep_us(5)
                self.step.low()
                time.sleep_us(loopTimeUs)

            # Decellerate
            for i in range(0, dSteps):
                self.step.high()
                time.sleep_us(5)
                self.step.low()
                multiplier = dM * i + 1
                time.sleep_us(round(loopTimeUs * multiplier))
            
            self.setCurrentMilliamps(self.idleCurrent)
            return True
        else:
            print("Error:  Driver in termal warning")
            return False


    def moveSteps(self, steps, speed, direction, current, debug=False):
        """
        Moves the stepper motor without any acceleration/decelleration.
        Will fail if the driver is in thermal warning

        Parameters
        ----------
        steps : int
            The total number of steps to move
        speed : int
            The speed in Hz to move
        direction : boolean
        current : int
            The current in mA to apply to the motor when moving.  After the move the motor is returned to the idleCurrent
        debug : boolean
            If True will print debug output

        Returns
        -------
        boolean
            Indicates success or failure (depending on thermal warning)
        """
        if not self.readThermalWarning():
            if debug:
                print("Move Steps:", steps, " @ speed:", speed, " in dierction:", direction, " using", current, "mA")
            self.setDirection(direction)
            self.setCurrentMilliamps(current)
            loopTimeUs = round(((1/speed) * 1000000) - 10)
            for i in range(steps):
                self.step.high()
                time.sleep_us(10)
                self.step.low()
                time.sleep_us(loopTimeUs)
            self.setCurrentMilliamps(self.idleCurrent)
        else:
            print("Error:  Driver in termal warning")
