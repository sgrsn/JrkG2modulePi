import serial
from smbus2 import SMBus, i2c_msg
from enum import IntEnum, Enum
import ctypes

#not needed
import time

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

class JrkG2ForceMode(Enum):

  none            = 0,
  DutyCycleTarget = 1,
  DutyCycle       = 2,

class JrkG2Reset(Enum):

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

class JrkG2OptionsByte3(Enum):

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

class SettingOffset(Enum):
    
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
    
    def getCurrentChoppingOccurrenceCount(self)
        return self.get_variables(JrkG2Command.GetCurrentChoppingOccurrenceCount, 1)
    
    # RAM setting command
    
    def setResetIntegral(bool reset):
        uint8_t tmp = getRAMSetting8(SettingOffset::OptionsByte3);
        if (getLastError())  return; 
        if (reset)

          tmp |= 1 << (uint8_t)JrkG2OptionsByte3::ResetIntegral;

        else

          tmp &= ~(1 << (uint8_t)JrkG2OptionsByte3::ResetIntegral);

        setRAMSetting8(SettingOffset::OptionsByte3, tmp);

    def getResetIntegral():

        return getRAMSetting8(SettingOffset::OptionsByte3) >> (uint8_t)JrkG2OptionsByte3::ResetIntegral & 1;
    
    def setCoastWhenOff(bool coast)

        uint8_t tmp = getRAMSetting8(SettingOffset::OptionsByte3);
        if (getLastError())  return; 
        if (coast):

            tmp |= 1 << (uint8_t)JrkG2OptionsByte3::CoastWhenOff;

        else

            tmp &= ~(1 << (uint8_t)JrkG2OptionsByte3::CoastWhenOff);

        setRAMSetting8(SettingOffset::OptionsByte3, tmp);
  

  /// Gets the "Coast when off" setting from the Jrk's RAM settings.
  ///
  /// See also setCoastWhenOff().
  bool getCoastWhenOff()
  
    return getRAMSetting8(SettingOffset::OptionsByte3) >>
      (uint8_t)JrkG2OptionsByte3::CoastWhenOff & 1;
  

  /// Sets the proportional coefficient in the Jrk's RAM settings.
  ///
  /// This coefficient is used in the Jrk's PID algorithm.  The coefficient
  /// takes the form:
  ///
  /// multiplier / (2 ^ exponent)
  ///
  /// The multiplier can range from 0 to 1023, and the exponent can range
  /// from 0 to 18.
  ///
  /// Example usage:
  /// ```
  /// // Set the proportional coefficient to 1.125 (9/(2^3)).
  /// jrk.setProportionalCoefficient(9, 3);
  /// ```
  ///
  /// You would normally configure this setting ahead of time using the Jrk G2
  /// Configuration Utility, but this function allows you to change it
  /// temporarily on the fly.
  ///
  /// See also getProportionalMultiplier() and getProportionalExponent(), as
  /// well as setIntegralCoefficient() and setDerivativeCoefficient().
  void setProportionalCoefficient(uint16_t multiplier, uint8_t exponent)
  
    setPIDCoefficient(SettingOffset::ProportionalMultiplier, multiplier, exponent);
  

  /// Gets the multiplier part of the proportional coefficient from the Jrk's
  /// RAM settings.
  ///
  /// See also getProportionalExponent() and setProportionalCoefficient().
  uint16_t getProportionalMultiplier()
  
    return getRAMSetting16(SettingOffset::ProportionalMultiplier);
  

  /// Gets the exponent part of the proportional coefficient from the Jrk's RAM
  /// settings.
  ///
  /// See also getProportionalMultiplier() and setProportionalCoefficient().
  uint8_t getProportionalExponent()
  
    return getRAMSetting8(SettingOffset::ProportionalExponent);
  

  /// Sets the integral coefficient in the Jrk's RAM settings.
  ///
  /// This coefficient is used in the Jrk's PID algorithm.  The coefficient
  /// takes the form:
  ///
  /// multiplier / (2 ^ exponent)
  ///
  /// The multiplier can range from 0 to 1023, and the exponent can range
  /// from 0 to 18.
  ///
  /// You would normally configure this setting ahead of time using the Jrk G2
  /// Configuration Utility, but this function allows you to change it
  /// temporarily on the fly.
  ///
  /// See also getIntegralMultiplier() and getIntegralExponent(), as
  /// well as setProportionalCoefficient() and setDerivativeCoefficient().
  void setIntegralCoefficient(uint16_t multiplier, uint8_t exponent)
  
    setPIDCoefficient(SettingOffset::IntegralMultiplier, multiplier, exponent);
  

  /// Gets the multiplier part of the integral coefficient from the Jrk's
  /// RAM settings.
  ///
  /// See also getIntegralExponent() and setIntegralCoefficient().
  uint16_t getIntegralMultiplier()
  
    return getRAMSetting16(SettingOffset::IntegralMultiplier);
  

  /// Gets the exponent part of the integral coefficient from the Jrk's
  /// RAM settings.
  ///
  /// See also getIntegralMultiplier() and setIntegralCoefficient().
  uint8_t getIntegralExponent()
  
    return getRAMSetting8(SettingOffset::IntegralExponent);
  

  /// Sets the derivative coefficient in the Jrk's RAM settings.
  ///
  /// This coefficient is used in the Jrk's PID algorithm.  The coefficient
  /// takes the form:
  ///
  /// multiplier / (2 ^ exponent)
  ///
  /// The multiplier can range from 0 to 1023, and the exponent can range
  /// from 0 to 18.
  ///
  /// You would normally configure this setting ahead of time using the Jrk G2
  /// Configuration Utility, but this function allows you to change it
  /// temporarily on the fly.
  ///
  /// See also getDerivativeMultiplier() and getDerivativeExponent(), as
  /// well as setProportionalCoefficient() and setIntegralCoefficient().
  void setDerivativeCoefficient(uint16_t multiplier, uint8_t exponent)
  
    setPIDCoefficient(SettingOffset::DerivativeMultiplier, multiplier, exponent);
  

  /// Gets the multiplier part of the derivative coefficient from the
  /// Jrk's RAM settings.
  ///
  /// See also getDerivativeExponent() and setDerivativeCoefficient().
  uint16_t getDerivativeMultiplier()
  
    return getRAMSetting16(SettingOffset::DerivativeMultiplier);
  

  /// Gets the exponent part of the derivative coefficient from the
  /// Jrk's RAM settings.
  ///
  /// See also getDerivativeMultiplier() and setDerivativeCoefficient().
  uint8_t getDerivativeExponent()
  
    return getRAMSetting8(SettingOffset::DerivativeExponent);
  

  /// Sets the PID period in the Jrk's RAM settings.
  ///
  /// This is the rate at which the Jrk runs through all of its calculations, in
  /// milliseconds.  Note that a higher PID period will result in a more slowly
  /// changing integral and a higher derivative, so the two corresponding PID
  /// coefficients might need to be adjusted whenever the PID period is changed.
  ///
  /// You would normally configure this setting ahead of time using the Jrk G2
  /// Configuration Utility, but this function allows you to change it
  /// temporarily on the fly.
  ///
  /// See also getPIDPeriod().
  void setPIDPeriod(uint16_t period)
  
    setRAMSetting16(SettingOffset::PIDPeriod, period);
  

  /// Gets the PID period from the Jrk's RAM settings, in milliseconds.
  ///
  /// See also setPIDPeriod().
  uint16_t getPIDPeriod()
  
    return getRAMSetting16(SettingOffset::PIDPeriod);
  

  /// Sets the integral limit in the Jrk's RAM settings.
  ///
  /// The PID algorithm prevents the absolute value of the integral variable
  /// (also known as error sum) from exceeding this limit.  This can help limit
  /// integral wind-up.  The limit can range from 0 to 32767.
  ///
  /// You would normally configure this setting ahead of time using the Jrk G2
  /// Configuration Utility, but this function allows you to change it
  /// temporarily on the fly.
  ///
  /// See also getIntegralLimit().
  void setIntegralLimit(uint16_t limit)
  
    setRAMSetting16(SettingOffset::IntegralLimit, limit);
  

  /// Gets the integral limit from the Jrk's RAM settings.
  ///
  /// See also setIntegralLimit().
  uint16_t getIntegralLimit()
  
    return getRAMSetting16(SettingOffset::IntegralLimit);
  

  /// Sets the maximum duty cycle while feedback is out of range in the Jrk's
  /// RAM settings.
  ///
  /// You would normally configure this setting ahead of time using the Jrk G2
  /// Configuration Utility, but this function allows you to change it
  /// temporarily on the fly.
  ///
  /// See also getMaxDutyCycleWhileFeedbackOutOfRange().
  void setMaxDutyCycleWhileFeedbackOutOfRange(uint16_t duty)
  
    setRAMSetting16(SettingOffset::MaxDutyCycleWhileFeedbackOutOfRange, duty);
  

  /// Gets the maximum duty cycle while feedback is out of range from the Jrk's RAM
  /// settings.
  ///
  /// See also setMaxDutyCycleWhileFeedbackOutOfRange().
  uint16_t getMaxDutyCycleWhileFeedbackOutOfRange()
  
    return getRAMSetting16(SettingOffset::MaxDutyCycleWhileFeedbackOutOfRange);
  

  /// Sets the maximum acceleration in the forward direction in the
  /// Jrk's RAM settings.
  ///
  /// You would normally configure this setting ahead of time using the Jrk G2
  /// Configuration Utility, but this function allows you to change it
  /// temporarily on the fly.
  ///
  /// See also getMaxAccelerationForward(), setMaxAccelerationReverse(),
  /// setMaxAcceleration(), and setMaxDecelerationForward().
  void setMaxAccelerationForward(uint16_t accel)
  
    setRAMSetting16(SettingOffset::MaxAccelerationForward, accel);
  

  /// Gets the maximum acceleration in the forward direction from the
  /// Jrk's RAM settings.
  ///
  /// See also setMaxAccelerationForward().
  uint16_t getMaxAccelerationForward()
  
    return getRAMSetting16(SettingOffset::MaxAccelerationForward);
  

  /// Sets the maximum acceleration in the reverse direction in the
  /// Jrk's RAM settings.
  ///
  /// You would normally configure this setting ahead of time using the Jrk G2
  /// Configuration Utility, but this function allows you to change it
  /// temporarily on the fly.
  ///
  /// See also getMaxAccelerationReverse(), setMaxAccelerationForward(),
  /// setMaxAcceleration(), and setMaxDecelerationReverse().
  void setMaxAccelerationReverse(uint16_t accel)
  
    setRAMSetting16(SettingOffset::MaxAccelerationReverse, accel);
  

  /// Gets the maximum acceleration in the reverse direction from the
  /// Jrk's RAM settings.
  ///
  /// See also setMaxAccelerationReverse().
  uint16_t getMaxAccelerationReverse()
  
    return getRAMSetting16(SettingOffset::MaxAccelerationReverse);
  

  /// Sets the maximum acceleration in both directions in the
  /// Jrk's RAM settings.
  ///
  /// You would normally configure this setting ahead of time using the Jrk G2
  /// Configuration Utility, but this function allows you to change it
  /// temporarily on the fly.
  ///
  /// See also setMaxAccelerationForward(), setMaxAccelerationReverse(),
  /// setMaxDeceleration().
  void setMaxAcceleration(uint16_t accel)
  
    setRAMSetting16x2(SettingOffset::MaxAccelerationForward, accel, accel);
  

  /// Sets the maximum deceleration in the forward direction in the Jrk's RAM
  /// settings.
  ///
  /// You would normally configure this setting ahead of time using the Jrk G2
  /// Configuration Utility, but this function allows you to change it
  /// temporarily on the fly.
  ///
  /// See also getMaxDecelerationForward(), setMaxDecelerationReverse(),
  /// setMaxDeceleration(), and setMaxAccelerationForward().
  void setMaxDecelerationForward(uint16_t decel)
  
    setRAMSetting16(SettingOffset::MaxDecelerationForward, decel);
  

  /// Gets the maximum deceleration in the forward direction from the Jrk's RAM
  /// settings.
  ///
  /// See also setMaxDecelerationForward().
  uint16_t getMaxDecelerationForward()
  
    return getRAMSetting16(SettingOffset::MaxDecelerationForward);
  

  /// Sets the maximum deceleration in the reverse direction in the
  /// Jrk's RAM settings.
  ///
  /// You would normally configure this setting ahead of time using the Jrk G2
  /// Configuration Utility, but this function allows you to change it
  /// temporarily on the fly.
  ///
  /// See also getMaxDecelerationReverse(), setMaxDecelerationForward(),
  /// setMaxDeceleration(), and setMaxAccelerationReverse().
  void setMaxDecelerationReverse(uint16_t decel)
  
    setRAMSetting16(SettingOffset::MaxDecelerationReverse, decel);
  

  /// Gets the maximum deceleration in the reverse direction from the Jrk's RAM
  /// settings.
  ///
  /// See also setMaxDecelerationReverse().
  uint16_t getMaxDecelerationReverse()
  
    return getRAMSetting16(SettingOffset::MaxDecelerationReverse);
  

  /// Sets the maximum deceleration in both directions in the
  /// Jrk's RAM settings.
  ///
  /// You would normally configure this setting ahead of time using the Jrk G2
  /// Configuration Utility, but this function allows you to change it
  /// temporarily on the fly.
  ///
  /// See also setMaxDecelerationForward(), setMaxDecelerationReverse(),
  /// setMaxAcceleration().
  void setMaxDeceleration(uint16_t decel)
  
    setRAMSetting16x2(SettingOffset::MaxDecelerationForward, decel, decel);
  

  /// Sets the maximum duty cycle in the forward direction in the
  /// Jrk's RAM settings.
  ///
  /// You would normally configure this setting ahead of time using the Jrk G2
  /// Configuration Utility, but this function allows you to change it
  /// temporarily on the fly.
  ///
  /// See also getMaxDutyCycleForward(), setMaxDutyCycleReverse().
  void setMaxDutyCycleForward(uint16_t duty)
  
    setRAMSetting16(SettingOffset::MaxDutyCycleForward, duty);
  

  /// Gets the maximum duty cycle in the forward direction from the Jrk's RAM
  /// settings.
  ///
  /// See also setMaxDutyCycleForward().
  uint16_t getMaxDutyCycleForward()
  
    return getRAMSetting16(SettingOffset::MaxDutyCycleForward);
  

  /// Sets the maximum duty cycle in the reverse direction in the
  /// Jrk's RAM settings.
  ///
  /// You would normally configure this setting ahead of time using the Jrk G2
  /// Configuration Utility, but this function allows you to change it
  /// temporarily on the fly.
  ///
  /// See also getMaxDutyCycleReverse(), setMaxDutyCycleForard().
  void setMaxDutyCycleReverse(uint16_t duty)
  
    setRAMSetting16(SettingOffset::MaxDutyCycleReverse, duty);
  

  /// Gets the maximum duty cycle in the reverse direction from the
  /// Jrk's RAM settings.
  ///
  /// See also setMaxDutyCycleReverse().
  uint16_t getMaxDutyCycleReverse()
  
    return getRAMSetting16(SettingOffset::MaxDutyCycleReverse);
  

  /// Sets the maximum duty cycle for both directions in the
  /// Jrk's RAM settings.
  ///
  /// You would normally configure this setting ahead of time using the Jrk G2
  /// Configuration Utility, but this function allows you to change it
  /// temporarily on the fly.
  ///
  /// See also setMaxDutyCycleForward(), setMaxDutyCycleReverse().
  void setMaxDutyCycle(uint16_t duty)
  
    setRAMSetting16x2(SettingOffset::MaxDutyCycleForward, duty, duty);
  

  /// Sets the encoded hard current limit for driving in the forward direction
  /// in the Jrk's RAM settings.
  ///
  /// You would normally configure this setting ahead of time using the Jrk G2
  /// Configuration Utility, but this function allows you to change it
  /// temporarily on the fly.
  ///
  /// This command is only valid for the Jrk G2 18v19, 24v13, 18v27, and 24v21.
  /// The Jrk G2 21v3 does not have a configurable hard current limit.
  ///
  /// See also getEncodedHardCurrentLimitForward() and
  /// setEncodedHardCurrentLimitReverse().
  void setEncodedHardCurrentLimitForward(uint16_t encoded_limit)
  
    setRAMSetting16(SettingOffset::EncodedHardCurrentLimitForward,
      encoded_limit);
  

  /// Gets the encoded hard current limit for driving in the forward direction
  /// from the Jrk's RAM settings.
  ///
  /// This command is only valid for the Jrk G2 18v19, 24v13, 18v27, and 24v21.
  /// The Jrk G2 21v3 does not have a configurable hard current limit.
  ///
  /// See also setEncodedHardCurrentLimitForward().
  uint16_t getEncodedHardCurrentLimitForward()
  
    return getRAMSetting16(SettingOffset::EncodedHardCurrentLimitForward);
  

  /// Sets the encoded hard current limit for driving in the reverse direction
  /// in the Jrk's RAM settings
  ///
  /// You would normally configure this setting ahead of time using the Jrk G2
  /// Configuration Utility, but this function allows you to change it
  /// temporarily on the fly.
  ///
  /// This command is only valid for the Jrk G2 18v19, 24v13, 18v27, and 24v21.
  /// The Jrk G2 21v3 does not have a configurable hard current limit.
  ///
  /// See also getEncodedHardCurrentLimitReverse() and
  /// setEncodedHardCurrentLimitForward().
  void setEncodedHardCurrentLimitReverse(uint16_t encoded_limit)
  
    setRAMSetting16(SettingOffset::EncodedHardCurrentLimitReverse, encoded_limit);
  

  /// Gets the encoded hard current limit for driving in the reverse direction
  /// from the Jrk's RAM settings.
  ///
  /// This command is only valid for the Jrk G2 18v19, 24v13, 18v27, and 24v21.
  /// The Jrk G2 21v3 does not have a configurable hard current limit.
  ///
  /// See also setEncodedHardCurrentLimitReverse().
  uint16_t getEncodedHardCurrentLimitReverse()
  
    return getRAMSetting16(SettingOffset::EncodedHardCurrentLimitReverse);
  

  /// Sets the encoded hard current limit for both directions in the Jrk's RAM
  /// settings.
  ///
  /// You would normally configure this setting ahead of time using the Jrk G2
  /// Configuration Utility, but this function allows you to change it
  /// temporarily on the fly.
  ///
  /// This command is only valid for the Jrk G2 18v19, 24v13, 18v27, and 24v21.
  /// The Jrk G2 21v3 does not have a configurable hard current limit.
  ///
  /// See also setEncodedHardCurrentLimitForward(),
  /// setEncodedHardCurrentLimitReverse(), getEncodedHardCurrentLimit(), and
  /// setSoftCurrentLimit().
  void setEncodedHardCurrentLimit(uint16_t encoded_limit)
  
    setRAMSetting16x2(SettingOffset::EncodedHardCurrentLimitForward,
      encoded_limit, encoded_limit);
  

  /// Sets the brake duration when switching from forward to reverse in the
  /// Jrk's RAM settings, in units of 5 ms.
  ///
  /// You would normally configure this setting ahead of time using the Jrk G2
  /// Configuration Utility, but this function allows you to change it
  /// temporarily on the fly.
  ///
  /// See also getBrakeDurationForward() and setBrakeDurationReverse().
  void setBrakeDurationForward(uint8_t duration)
  
    setRAMSetting8(SettingOffset::BrakeDurationForward, duration);
  

  /// Gets the brake duration when switching from forward to reverse from the
  /// Jrk's RAM settings, in units of 5 ms.
  ///
  /// See also setBrakeDurationForward().
  uint8_t getBrakeDurationForward()
  
    return getRAMSetting8(SettingOffset::BrakeDurationForward);
  

  /// Sets the brake duration when switching from reverse to forward in the
  /// Jrk's RAM settings, in units of 5 ms.
  ///
  /// You would normally configure this setting ahead of time using the Jrk G2
  /// Configuration Utility, but this function allows you to change it
  /// temporarily on the fly.
  ///
  /// See also getBrakeDurationReverse() and setBrakeDurationForward().
  void setBrakeDurationReverse(uint8_t duration)
  
    setRAMSetting8(SettingOffset::BrakeDurationReverse, duration);
  

  /// Gets the brake duration when switching from reverse to forward from the
  /// Jrk's RAM settings, in units of 5 ms.
  ///
  /// See also setBrakeDurationReverse().
  uint8_t getBrakeDurationReverse()
  
    return getRAMSetting8(SettingOffset::BrakeDurationReverse);
  

  /// Sets the brake duration for both directions in the Jrk's RAM settings, in
  /// units of 5 ms.
  ///
  /// You would normally configure this setting ahead of time using the Jrk G2
  /// Configuration Utility, but this function allows you to change it
  /// temporarily on the fly.
  ///
  /// See also setBrakeDurationForward(), setBrakeDurationReverse().
  void setBrakeDuration(uint8_t duration)
  
    setRAMSetting8x2(SettingOffset::BrakeDurationForward, duration, duration);
  

  /// Sets the soft current limit when driving in the forward direction in the
  /// Jrk's RAM settings, in units of mA.
  ///
  /// You would normally configure this setting ahead of time using the Jrk G2
  /// Configuration Utility, but this function allows you to change it
  /// temporarily on the fly.
  ///
  /// See also getSoftCurrentLimitForward() and setSoftCurrentLimitReverse().
  void setSoftCurrentLimitForward(uint16_t current)
  
    setRAMSetting16(SettingOffset::SoftCurrentLimitForward, current);
  

  /// Gets the soft current limit when driving in the forward direction from the
  /// Jrk's RAM settings, in units of mA.
  ///
  /// See also setSoftCurrentLimitForward().
  uint16_t getSoftCurrentLimitForward()
  
    return getRAMSetting16(SettingOffset::SoftCurrentLimitForward);
  

  /// Sets the soft current limit when driving in the reverse direction in the
  /// Jrk's RAM settings, in units of mA.
  ///
  /// You would normally configure this setting ahead of time using the Jrk G2
  /// Configuration Utility, but this function allows you to change it
  /// temporarily on the fly.
  ///
  /// See also getSoftCurrentLimitReverse() and setSoftCurrentLimitForward().
  void setSoftCurrentLimitReverse(uint16_t current)
  
    setRAMSetting16(SettingOffset::SoftCurrentLimitReverse, current);
  

  /// Gets the soft current limit when driving in the reverse direction from the
  /// Jrk's RAM settings, in units of mA.
  ///
  /// See also setSoftCurrentLimitReverse().
  uint16_t getSoftCurrentLimitReverse()
  
    return getRAMSetting16(SettingOffset::SoftCurrentLimitReverse);
  

  /// Sets the soft current limit for driving in both directions in the Jrk's
  /// RAM settings, in units of mA.
  ///
  /// You would normally configure this setting ahead of time using the Jrk G2
  /// Configuration Utility, but this function allows you to change it
  /// temporarily on the fly.
  ///
  /// See also setSoftCurrentLimitForward() and setSoftCurrentLimitReverse(),
  /// setEncodedHardCurrentLimit().
  void setSoftCurrentLimit(uint16_t current)
  
    setRAMSetting16x2(SettingOffset::SoftCurrentLimitForward, current, current);
  

    
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
    
def serialTest():

    # you can run "jrk2cmd --cmd-port" to get the right name to use here.
    # Linux USB example:  "/dev/ttyACM0"
    # macOS USB example:  "/dev/cu.usbmodem001234562"
    # Windows example:    "COM6"
    port_name = "/dev/ttyACM0"

    baud_rate = 9600
    device_number = None

    port = serial.Serial(port_name, baud_rate, timeout=0.1, write_timeout=0.1)
    jrk = JrkG2Serial(port, device_number)
     
    feedback = jrk.getFeedback()
    print("Feedback is .".format(feedback))
    target = jrk.getTarget()
     
    new_target = 2248 if target < 2048 else 1848
    #jrk.setTarget(new_target)
    #jrk.forceDutyCycleTarget(600)
    
def i2cTest():
    
    bus = SMBus(1)
    address = 11
     
    jrk = JrkG2I2C(bus, address)
     
    feedback = jrk.getFeedback()
    print("Feedback is .".format(feedback))
     
    target = jrk.getTarget()
    print("Target is .".format(target))
     
    new_target = 2248 if target < 2048 else 1855
    print("Setting target to .".format(new_target))
    #jrk.setTarget(new_target)
    jrk.forceDutyCycleTarget(600)
    time.sleep(1)
    jrk.stopMotor()

if __name__ == '__main__':
    #serialTest()
    i2cTest()