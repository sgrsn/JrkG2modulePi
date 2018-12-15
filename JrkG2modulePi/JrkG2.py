import serial
from smbus2 import SMBus, i2c_msg
from enum import IntEnum
import ctypes

# This is used to represent a null or missing value for some of the Jrk G2's
# 16-bit input variables.
JrkG2InputNull = 0xFFFF

# This value is returned by getLastError() if the last communication with the
# device resulted in an unsuccessful read (e.g. timeout or NACK).
JrkG2CommReadError = 50

class JrkG2Error(IntEnum):
    
  AwaitingCommand     = 0,
  NoPower             = 1,
  MotorDriver         = 2,
  InputInvalid        = 3,
  InputDisconnect     = 4,
  FeedbackDisconnect  = 5,
  SoftOvercurrent     = 6,
  SerialSignal        = 7,
  SerialOverrun       = 8,
  SerialBufferFull    = 9,
  SerialCrc           = 10,
  SerialProtocol      = 11,
  SerialTimeout       = 12,
  HardOvercurrent     = 13,

class JrkG2Command(IntEnum):
    
  SetTarget                         = 0xC0,
  SetTargetLowResRev                = 0xE0,
  SetTargetLowResFwd                = 0xE1,
  ForceDutyCycleTarget              = 0xF2,
  ForceDutyCycle                    = 0xF4,
  MotorOff                          = 0xFF,
  GetVariable8                      = 0x80,
  GetVariable16                     = 0xA0,
  GetEEPROMSettings                 = 0xE3,
  GetVariables                      = 0xE5,
  SetRAMSettings                    = 0xE6,
  GetRAMSettings                    = 0xEA,
  GetCurrentChoppingOccurrenceCount = 0xEC,

class JrkG2ForceMode(IntEnum):

  none            = 0,
  DutyCycleTarget = 1,
  DutyCycle       = 2,

class JrkG2Reset(IntEnum):

  PowerUp        = 0,
  Brownout       = 1,
  ResetLine      = 2,
  Watchdog       = 4,
  Software       = 8,
  StackOverflow  = 16,
  StackUnderflow = 32,

class JrkG2Pin(IntEnum):

  SCL = 0,
  SDA = 1,
  TX  = 2,
  RX  = 3,
  RC  = 4,
  AUX = 5,
  FBA = 6,
  FBT = 7,

class JrkG2OptionsByte3(IntEnum):

  ResetIntegral = 0,
  CoastWhenOff = 1,
  
class VarOffset(IntEnum):
    
    Input                           = 0x00, # uint16_t
    Target                          = 0x02, # uint16_t
    Feedback                        = 0x04, # uint16_t
    ScaledFeedback                  = 0x06, # uint16_t
    Integral                        = 0x08, # int16_t
    DutyCycleTarget                 = 0x0A, # int16_t
    DutyCycle                       = 0x0C, # int16_t
    CurrentLowRes                   = 0x0E, # uint8_t
    PIDPeriodExceeded               = 0x0F, # uint8_t
    PIDPeriodCount                  = 0x10, # uint16_t
    ErrorFlagsHalting               = 0x12, # uint16_t
    ErrorFlagsOccurred              = 0x14, # uint16_t

    FlagByte1                       = 0x16, # uint8_t
    VinVoltage                      = 0x17, # uint16_t
    Current                         = 0x19, # uint16_t

    # variables above can be read with single-byte commands (GetVariable)
    # variables below must be read with segment read (GetVariables)

    DeviceReset                     = 0x1F, # uint8_t
    UpTime                          = 0x20, # uint32_t
    RCPulseWidth                    = 0x24, # uint16_t
    FBTReading                      = 0x26, # uint16_t
    AnalogReadingSDA                = 0x28, # uint16_t
    AnalogReadingFBA                = 0x2A, # uint16_t
    DigitalReadings                 = 0x2C, # uint8_t
    RawCurrent                      = 0x2D, # uint16_t
    EncodedHardCurrentLimit         = 0x2F, # uint16_t
    LastDutyCycle                   = 0x31, # int16_t
    CurrentChoppingConsecutiveCount = 0x33, # uint8_t
    CurrentChoppingOccurrenceCount  = 0x34, # uint8_t; read with dedicated command

class SettingOffset(IntEnum):
    
    OptionsByte1                        = 0x01,  # uint8_t
    OptionsByte2                        = 0x02,  # uint8_t
    InputMode                           = 0x03,  # uint8_t
    InputErrorMinimum                   = 0x04,  # uint16_t,
    InputErrorMaximum                   = 0x06,  # uint16_t,
    InputMinimum                        = 0x08,  # uint16_t,
    InputMaximum                        = 0x0A,  # uint16_t,
    InputNeutralMinimum                 = 0x0C,  # uint16_t,
    InputNeutralMaximum                 = 0x0E,  # uint16_t,
    OutputMinimum                       = 0x10,  # uint16_t,
    OutputNeutral                       = 0x12,  # uint16_t,
    OutputMaximum                       = 0x14,  # uint16_t,
    InputScalingDegree                  = 0x16,  # uint8_t,
    InputAnalogSamplesExponent          = 0x17,  # uint8_t,
    FeedbackMode                        = 0x18,  # uint8_t,
    FeedbackErrorMinimum                = 0x19,  # uint16_t,
    FeedbackErrorMaximum                = 0x1B,  # uint16_t,
    FeedbackMinimum                     = 0x1D,  # uint16_t,
    FeedbackMaximum                     = 0x1F,  # uint16_t,
    FeedbackDeadZone                    = 0x21,  # uint8_t,
    FeedbackAnalogSamplesExponent       = 0x22,  # uint8_t,
    SerialMode                          = 0x23,  # uint8_t,
    SerialBaudRateGenerator             = 0x24,  # uint16_t,
    SerialTimeout                       = 0x26,  # uint16_t,
    SerialDeviceNumber                  = 0x28,  # uint16_t,
    ErrorEnable                         = 0x2A,  # uint16_t
    ErrorLatch                          = 0x2C,  # uint16_t
    ErrorHard                           = 0x2E,  # uint16_t
    VinCalibration                      = 0x30,  # uint16_t
    PwmFrequency                        = 0x32,  # uint8_t
    CurrentSamplesExponent              = 0x33,  # uint8_t
    HardOvercurrentThreshold            = 0x34,  # uint8_t
    CurrentOffsetCalibration            = 0x35,  # uint16_t
    CurrentScaleCalibration             = 0x37,  # uint16_t
    FBTMethod                           = 0x39,  # uint8_t
    FBTOptions                          = 0x3A,  # uint8_t
    FBTTimingTimeout                    = 0x3B,  # uint16_t
    FBTSamples                          = 0x3D,  # uint8_t
    FBTDividerExponent                  = 0x3E,  # uint8_t
    IntegralDividerExponent             = 0x3F,  # uint8_t
    SoftCurrentRegulationLevelForward   = 0x40,  # uint16_t
    SoftCurrentRegulationLevelReverse   = 0x42,  # uint16_t
    OptionsByte3                        = 0x50,  # uint8_t
    ProportionalMultiplier              = 0x51,  # uint16_t
    ProportionalExponent                = 0x53,  # uint8_t
    IntegralMultiplier                  = 0x54,  # uint16_t
    IntegralExponent                    = 0x56,  # uint8_t
    DerivativeMultiplier                = 0x57,  # uint16_t
    DerivativeExponent                  = 0x59,  # uint8_t
    PIDPeriod                           = 0x5A,  # uint16_t
    IntegralLimit                       = 0x5C,  # uint16_t
    MaxDutyCycleWhileFeedbackOutOfRange = 0x5E,  # uint16_t
    MaxAccelerationForward              = 0x60,  # uint16_t
    MaxAccelerationReverse              = 0x62,  # uint16_t
    MaxDecelerationForward              = 0x64,  # uint16_t
    MaxDecelerationReverse              = 0x66,  # uint16_t
    MaxDutyCycleForward                 = 0x68,  # uint16_t
    MaxDutyCycleReverse                 = 0x6A,  # uint16_t
    EncodedHardCurrentLimitForward      = 0x6C,  # uint16_t
    EncodedHardCurrentLimitReverse      = 0x6E,  # uint16_t
    BrakeDurationForward                = 0x70,  # uint8_t
    BrakeDurationReverse                = 0x71,  # uint8_t
    SoftCurrentLimitForward             = 0x72,  # uint16_t
    SoftCurrentLimitReverse             = 0x74,  # uint16_t
  
class JrkG2Base():
    
    def getLastError():
        return self._lastError
    
    # Motor control commands

    def setTarget(self, target):
        self.send_command(JrkG2Command.SetTarget | (target & 0x1F), (target >> 5))
    
    def setTargetLowResRev(self, target):
        if target > 127:
            target = 127
        self.send_command(JrkG2Command.SetTargetLowResRev, target)
        
    def setTargetLowResFwd(self, target):
        if target > 127:
            target = 127
        self.send_command(JrkG2Command.SetTargetLowResFwd, target)
        
    def forceDutyCycleTarget(self,  dutyCycle):
        if dutyCycle > 600:
            dutyCycle = 600
        if dutyCycle < -600:
            dutyCycle = -600
        self.send_commandWs14(JrkG2Command.ForceDutyCycleTarget, dutyCycle)
        
    def forceDutyCycle(self, dutyCycle):
        if dutyCycle > 600:
            dutyCycle = 600
        if dutyCycle < -600:
            dutyCycle = -600
        self.send_commandWs14(JrkG2Command.ForceDutyCycle, dutyCycle)
        
    def stopMotor(self):
        self.commandQuick(JrkG2Command.MotorOff)
        

    # Variable reading command
    
    def sign16bits(self, val):
        return -(val & 0b1000000000000000) | (val & 0b111111111111111)
    
    def getInput(self):
        return self.get_variables(VarOffset.Input, 2)

    def getTarget(self):
        return self.get_variables(VarOffset.Target, 2)

    def getFeedback(self):
        return self.get_variables(VarOffset.Feedback, 2)
    
    def getScaledFeedback(self):
        return self.get_variables(VarOffset.ScaledFeedback, 2)
    
    def getIntegral(self):
        val = self.get_variables(VarOffset.Integral, 2)
        return self.sign16bits(val)
    
    def getDutyCycleTarget(self):
        val = self.get_variables(VarOffset.DutyCycleTarget, 2)
        return self.sign16bits(val)
    
    def getDutyCycle(self):
        val = self.get_variables(VarOffset.DutyCycle, 2)
        return self.sign16bits(val)
    
    def getCurrentLowRes(self):
        return self.get_variables(VarOffset.DutyCycle, 1)
    
    def getPIDPeriodExceeded(self):
        return self.get_variables(VarOffset.PIDPeriodExceeded, 1)
    
    def getPIDPeriodCount(self):
        return self.get_variables(VarOffset.PIDPeriodCount, 2)
    
    def getErrorFlagsHalting(self):
        return self.get_variables(VarOffset.ErrorFlagsHalting, 2)
    
    def getErrorFlagsOccurred(self):
        return self.get_variables(VarOffset.ErrorFlagsOccurred, 2)
    
    def getForceMode(self):
        val = self.get_variables(VarOffset.FlagByte1, 1)
        return val & 0x03
    
    def getVinVoltage(self):
        return self.get_variables(VarOffset.VinVoltage, 2)
    
    def getCurrent(self):
        return self.get_variables(VarOffset.Current, 2)
    
    def getDeviceReset(self):
        return self.get_variables(VarOffset.DeviceReset, 1)
    
    def getUpTime(self):
        return self.get_variables(VarOffset.UpTime, 4)
    
    def getRCPulseWidth(self):
        return self.get_variables(VarOffset.RCPulseWidth, 2)
    
    def getFBTReading(self):
        return get_variables(VarOffset.FBTReading, 2)
    
    def getAnalogReading(self, pin):
        if pin == JrkG2Pin.SDA:
            return self.get_variables(VarOffset.AnalogReadingSDA, 2)
        elif pin == JrkG2Pin.FBA:
            return self.get_variables(VarOffset.AnalogReadingFBA, 2)
        else:
            return JrkG2InputNull
        
    def getDigitalReading(self, pin):
        readings = self.get_variables(VarOffset.DigitalReadings, 1)
        return (readings >> pin) & 1
    
    def getRawCurrent(self):
        return self.get_variables(VarOffset.RawCurrent, 2)
    
    def getEncodedHardCurrentLimit(self):
        return self.get_variables(VarOffset.EncodedHardCurrentLimit, 2)
    
    def getLastDutyCycle(self):
        return self.get_variables(VarOffset.LastDutyCycle, 2)
    
    def getCurrentChoppingConsecutiveCount(self):
        return self.get_variables(VarOffset.CurrentChoppingConsecutiveCount, 1)
    
    def getCurrentChoppingOccurrenceCount(self):
        return self.get_variables(JrkG2Command.GetCurrentChoppingOccurrenceCount, 1)
    
    
    
    
    # RAM setting command
    
    def setResetIntegral(self, reset):
        tmp = self.getRAMSetting8(SettingOffset.OptionsByte3)
        if (getLastError()):
            return
        if (reset):
          tmp |= 1 << JrkG2OptionsByte3.ResetIntegral
        else:
          tmp &= ~(1 << JrkG2OptionsByte3.ResetIntegral)
        self.setRAMSetting8(SettingOffset.OptionsByte3, tmp)

    def getResetIntegral(self):
        return self.getRAMSetting8(SettingOffset.OptionsByte3) >> JrkG2OptionsByte3.ResetIntegral & 1
    
    def setCoastWhenOff(self, coast):
        tmp = self.getRAMSetting8(SettingOffset.OptionsByte3)
        if (getLastError()):
            return
        if (coast == 1):
            tmp |= 1 << JrkG2OptionsByte3.CoastWhenOff
        else:
            tmp &= ~(1 << JrkG2OptionsByte3.CoastWhenOff)
        self.setRAMSetting8(SettingOffset.OptionsByte3, tmp)
  
    def getCoastWhenOff(self):
        return self.getRAMSetting8(SettingOffset.OptionsByte3) >> JrkG2OptionsByte3.CoastWhenOff & 1

    def setProportionalCoefficient(self, coefficient):
        multiplier, exponent = self.calcValueFromCoefficient(coefficient)
        self.setPIDCoefficient(SettingOffset.ProportionalMultiplier, multiplier, exponent)
    
    def getProportionalMultiplier(self):
          return self.getRAMSetting16(SettingOffset.ProportionalMultiplier) 

    def getProportionalExponent(self):
        return self.getRAMSetting8(SettingOffset.ProportionalExponent)
  
    def setIntegralCoefficient(self, coefficient):
        multiplier, exponent = self.calcValueFromCoefficient(coefficient)
        self.setPIDCoefficient(SettingOffset.IntegralMultiplier, multiplier, exponent)
  
    def getIntegralMultiplier(self): 
        return self.getRAMSetting16(SettingOffset.IntegralMultiplier)

    def getIntegralExponent(self): 
        return self.getRAMSetting8(SettingOffset.IntegralExponent)

    def setDerivativeCoefficient(self, coefficient):
        multiplier, exponent = self.calcValueFromCoefficient(coefficient)
        self.setPIDCoefficient(SettingOffset.DerivativeMultiplier, multiplier, exponent)

    def getDerivativeMultiplier(self):
        return self.getRAMSetting16(SettingOffset.DerivativeMultiplier)
  
    def getDerivativeExponent(self):
        return self.getRAMSetting8(SettingOffset.DerivativeExponent)

    def setPIDPeriod(self, period):
        self.setRAMSetting16(SettingOffset.PIDPeriod, period)
  
    def getPIDPeriod(self):
        return self.getRAMSetting16(SettingOffset.PIDPeriod)
  
    def setIntegralLimit(self, limit):
        self.setRAMSetting16(SettingOffset.IntegralLimit, limit)
  
    def getIntegralLimit(self):
        return self.getRAMSetting16(SettingOffset.IntegralLimit)
  
    def setMaxDutyCycleWhileFeedbackOutOfRange(self, duty):
        self.setRAMSetting16(SettingOffset.MaxDutyCycleWhileFeedbackOutOfRange, duty)

    def getMaxDutyCycleWhileFeedbackOutOfRange(self):
        return self.getRAMSetting16(SettingOffset.MaxDutyCycleWhileFeedbackOutOfRange)
  
    def setMaxAccelerationForward(self, accel):
        self.setRAMSetting16(SettingOffset.MaxAccelerationForward, accel)
  
    def getMaxAccelerationForward(self):
        return self.getRAMSetting16(SettingOffset.MaxAccelerationForward)
  
    def setMaxAccelerationReverse(self, accel):
        self.setRAMSetting16(SettingOffset.MaxAccelerationReverse, accel);
  
    def getMaxAccelerationReverse(self):
        return self.getRAMSetting16(SettingOffset.MaxAccelerationReverse);
  
    def setMaxAcceleration(self, accel):
        self.setRAMSetting16x2(SettingOffset.MaxAccelerationForward, accel, accel);
  
    def setMaxDecelerationForward(self, decel):
        self.setRAMSetting16(SettingOffset.MaxDecelerationForward, decel);
  
    def getMaxDecelerationForward(self):
        return self.getRAMSetting16(SettingOffset.MaxDecelerationForward)
  
    def setMaxDecelerationReverse(self, decel):
        self.setRAMSetting16(SettingOffset.MaxDecelerationReverse, decel)
  
    def getMaxDecelerationReverse(self):
        return self.getRAMSetting16(SettingOffset.MaxDecelerationReverse)
  
    def setMaxDeceleration(self, decel):
        self.setRAMSetting16x2(SettingOffset.MaxDecelerationForward, decel, decel)
  
    def setMaxDutyCycleForward(self, duty):
        self.setRAMSetting16(SettingOffset.MaxDutyCycleForward, duty)
  
    def getMaxDutyCycleForward(self):
        return self.getRAMSetting16(SettingOffset.MaxDutyCycleForward)
  
    def setMaxDutyCycleReverse(self, duty):
        self.setRAMSetting16(SettingOffset.MaxDutyCycleReverse, duty)
  
    def getMaxDutyCycleReverse(self):
        return self.getRAMSetting16(SettingOffset.MaxDutyCycleReverse)
  
    def setMaxDutyCycle(self, duty):
        self.setRAMSetting16x2(SettingOffset.MaxDutyCycleForward, duty, duty)

    def setEncodedHardCurrentLimitForward(self, encoded_limit):
        self.setRAMSetting16(SettingOffset.EncodedHardCurrentLimitForward, encoded_limit)
  
    def getEncodedHardCurrentLimitForward(self):
        return self.getRAMSetting16(SettingOffset.EncodedHardCurrentLimitForward)
  
    def setEncodedHardCurrentLimitReverse(self, encoded_limit):
        self.setRAMSetting16(SettingOffset.EncodedHardCurrentLimitReverse, encoded_limit)
  
    def getEncodedHardCurrentLimitReverse(self):
        return self.getRAMSetting16(SettingOffset.EncodedHardCurrentLimitReverse)

    def setEncodedHardCurrentLimit(self, encoded_limit):
        self.setRAMSetting16x2(SettingOffset.EncodedHardCurrentLimitForward, encoded_limit, encoded_limit)

    def setBrakeDurationForward(self, duration):
        self.setRAMSetting8(SettingOffset.BrakeDurationForward, duration)
  
    def getBrakeDurationForward(self):
        return self.getRAMSetting8(SettingOffset.BrakeDurationForward)
  
    def setBrakeDurationReverse(self, duration):
        self.setRAMSetting8(SettingOffset.BrakeDurationReverse, duration)
  
    def getBrakeDurationReverse(self):
        return self.getRAMSetting8(SettingOffset.BrakeDurationReverse)
  
    def setBrakeDuration(self, duration):
        self.setRAMSetting8x2(SettingOffset.BrakeDurationForward, duration, duration)
  
    def setSoftCurrentLimitForward(self, current):
        self.setRAMSetting16(SettingOffset.SoftCurrentLimitForward, current)
  
    def getSoftCurrentLimitForward(self):
        return self.getRAMSetting16(SettingOffset.SoftCurrentLimitForward)
  
    def setSoftCurrentLimitReverse(self, current):
        self.setRAMSetting16(SettingOffset.SoftCurrentLimitReverse, current)
  
    def getSoftCurrentLimitReverse(self): 
        return self.getRAMSetting16(SettingOffset.SoftCurrentLimitReverse)
  
    def setSoftCurrentLimit(self, current):
        self.setRAMSetting16x2(SettingOffset.SoftCurrentLimitForward, current, current)


    # Function to calculate multiplier and exponent from PID coefficient.
    # On Jrk, it is calculated as follows.
    # coefficient = multiplier / 2^exponent
    # And The limit of each value is 1023(multiplier), 18(exponent).
    def calcValueFromCoefficient(self, coefficient):
        multiplier = 0
        exponent = 0
        error = 0
        error_min = 1000
        tmp = coefficient
        i = 0
        while i <= 18:
            e = i
            m = int(coefficient*(2**e)+0.5)
            _coefficient = float(m)/(2.0**e)
            error = abs(coefficient - _coefficient)
            if error < error_min:
                error_min = error
                multiplier = m
                exponent = e
            tmp *= 2
            if tmp > 1023:
                break
            i+=1
        return multiplier, exponent
        
        
    
    # Low-level settings command
    
    #The three below are not used

    def getEEPROMSettings(self, offset, length):
        data = self.segmentRead()
        
    def getRAMSettings(self, offset, length):
        data = self.segmentRead(JrkG2Command.GetRAMSettings, offset, length)
        return data
    
    def setRAMSettings(self, offset, length, *buffer):
        self.segmentWrite(JrkG2Command.SetRAMSettings, offset, length, buffer)

    #used
        
    #Setting RAM command

    def setRAMSetting8(self, offset, val):
        self.segmentWrite(JrkG2Command.SetRAMSettings, offset, 1, val)
        
    def setRAMSetting16(self, offset, val):
        self.segmentWrite(JrkG2Command.SetRAMSettings, offset, 2, val&0xFF, val>>8)
        
    def setRAMSetting8x2(self, offset, val1, val2):
        self.segmentWrite(JrkG2Command.SetRAMSettings, offset, 2, val1, val2)
        
    def setRAMSetting16x2(self, offset, val1, val2):
        self.segmentWrite(JrkG2Command.SetRAMSettings, offset, 4, val1&0xFF, val1>>8, val2&0xFF, val2>>8)
        
    def setPIDCoefficient(self, offset, multiplier, exponent):
        self.segmentWrite(JrkG2Command.SetRAMSettings, offset, 3, multiplier, (multiplier >> 8) & 0xFF, exponent)
    
    #Getting RAM command

    def getRAMSetting8(self, offset):
        return self.segmentRead(JrkG2Command.GetRAMSettings, offset, 1)
    
    def getRAMSetting16(self, offset):
        data = self.segmentRead(JrkG2Command.GetRAMSettings, offset, 2)
        return data[0] + 256*data[1]
    
class JrkG2Serial(JrkG2Base):
    
    def __init__(self, port, device_number=None):
        self.port = port
        self.device_number = device_number
    
    def send_command(self, cmd, *data_bytes):
        if self.device_number == None:
            header = [cmd]  # Compact protocol
        else:
            header = [0xAA, device_number, cmd & 0x7F]  # Pololu protocol
        self.port.write(header + list(data_bytes))
        
    def send_commandWs14(self, cmd, val):
        data = ctypes.c_uint16(val).value
        self.send_command(cmd, data&0x7f, (data>>7)&0x7F,  (data>>14)&0x7F)
        
    def serialW7(self, val):
        self.port.write([val & 0x7F])
        
    def sendCommandHeader(self, cmd):
        if self.device_number == None:
            self.port.write([cmd])
        else:
            self.port.write([0xAA])
            self.serialW7(_deviceNumber)
            self.serialW7(cmd)
        
    def commandQuick(self, cmd):
        if self.device_number == None:
            header = [cmd]  # Compact protocol
        else:
            header = [0xAA, device_number, cmd & 0x7F]  # Pololu protocol
        self.port.write(header)
        
    def get_variables(self, offset, length):
        self.send_command(JrkG2Command.GetVariables, offset, length)
        result = self.port.read(length)
        if len(result) != length:
            self._lastError = JrkG2CommReadError
            raise RuntimeError("Expected to read  bytes, got ."
            .format(length, len(result)))
            return [0]
        self._lastError = 0
        data = bytearray(result)
        val = 0
        for i in range(length):
            val |= data[i] << i*8
        return val
    
    def segmentWrite(self, cmd, offset, length, *buffer):
        if length > 7:
            length = 7
        self.sendCommandHeader(cmd)
        self.serialW7(int(offset))
        self.serialW7(length)
        data = list(buffer)
        msbs = 0
        for i in range(length):
            self.serialW7(data[i])
            msbs |= (data[i] >> 7 & 1) << i
        self.serialW7(msbs)
        
    def segmentRead(self, cmd, offset, length):
        if length > 15:
            length = 15
        self.send_command(cmd, offset, length)
        result = self.port.read(length)
        if len(result) != length:
            self._lastError = JrkG2CommReadError
            return [0]
        self._lastError = 0
        data = bytearray(result)
        return data

class JrkG2I2C(JrkG2Base):
    
    def __init__(self, bus, device_address = 0x0B):
        self.bus = bus
        self.addr = device_address
        
    def send_command(self, cmd, *data_bytes):
        command = [cmd] + list(data_bytes)
        write = i2c_msg.write(self.addr, command)
        self.bus.i2c_rdwr(write)
        
    def send_commandWs14(self, cmd, val):
        data = ctypes.c_uint16(val).value
        self.send_command(cmd, data&0xFF, data>>8)
        
    def commandQuick(self, cmd):
        write = i2c_msg.write(self.addr, [cmd])
        self.bus.i2c_rdwr(write)
        
    def get_variables(self, offset, length):
        self.bus.write_i2c_block_data(self.addr, 0xE5, [offset])
        result = []
        for i in range(length):
            result += [self.bus.read_byte(self.addr)]
        self._lastError = 0
        data = bytearray(result)
        val = 0
        for i in range(length):
            val |= data[i] << i*8
        return val
    
    def segmentWrite(self, cmd, offset, length, *buffer):
        if(length > 13):
            length = 13
        self.bus.write_i2c_block_data(self.addr, cmd, [offset, length] + list(buffer))
        
    def segmentRead(self, cmd, offset, length):
        if length > 15:
            length = 15
        self.bus.write_i2c_block_data(self.addr, cmd, [offset])
        result = []
        for i in range(length):
            result += [self.bus.read_byte(self.addr)]
        self._lastError = 0
        return bytearray(result)
    