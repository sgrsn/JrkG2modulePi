import serial
from enum import IntEnum, Enum

#not needed
import time

# This is used to represent a null or missing value for some of the Jrk G2's
# 16-bit input variables.
rkG2InputNull = 0xFFFF

# This value is returned by getLastError() if the last communication with the
# device resulted in an unsuccessful read (e.g. timeout or NACK).
JrkG2CommReadError = 50

# This enum defines the Jrk's error bits.  See the "Error handling" section of
# the Jrk G2 user's guide for more information about what these errors mean.
#
# See JrkG2Base::getErrorFlagsHalting() and JrkG2Base::getErrorFlagsOccurred().
class JrkG2Error(Enum):
    
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

# This enum defines the Jrk G2 command bytes which are used for its serial and
# I2C interfaces.  These bytes are used by the library and you should not need
# to use them.
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

# This enum defines the modes in which the Jrk G2's duty cycle target or duty
# cycle, normally derived from the output of its PID algorithm, can be
# overridden with a forced value.
#
# See JrkG2Base::getForceMode(), JrkG2Base::forceDutyCycleTarget(), and
# JrkG2Base::forceDutyCycle().
class JrkG2ForceMode(Enum):

  none            = 0,
  DutyCycleTarget = 1,
  DutyCycle       = 2,


# This enum defines the possible causes of a full microcontroller reset for
# the Jrk G2.
#
# See JrkG2Base::getDeviceReset().
class JrkG2Reset(Enum):

  PowerUp        = 0,
  Brownout       = 1,
  ResetLine      = 2,
  Watchdog       = 4,
  Software       = 8,
  StackOverflow  = 16,
  StackUnderflow = 32,


# This enum defines the Jrk G2's control and feedback pins.
class JrkG2Pin(Enum):

  SCL = 0,
  SDA = 1,
  TX  = 2,
  RX  = 3,
  RC  = 4,
  AUX = 5,
  FBA = 6,
  FBT = 7,


# This enum defines the bits in the Jrk G2's Options Byte 3 register.  You
# should not need to use this directly.  See JrkG2Base::setResetIntegral(),
# JrkG2Base::getResetIntegral(), JrkG2Base::setCoastWhenOff(), and
# JrkG2Base::getCoastWhenOff().
class JrkG2OptionsByte3(Enum):

  ResetIntegral = 0,
  CoastWhenOff = 1,


 
class JrkG2Serial(object):
    
    def __init__(self, port, device_number=None):
        self.port = port
        self.device_number = device_number

    #commandW7(cmd, val) is equivalent to send_command(cmd, val)
    #commandWs14(cmd, val) is to send_command(cmd, val&0x7f, val>>7)
    def send_command(self, cmd, *data_bytes):
        if self.device_number == None:
            header = [cmd]  # Compact protocol
        else:
            header = [0xAA, device_number, cmd & 0x7F]  # Pololu protocol
        self.port.write(header + list(data_bytes))
        
    def send_commandWs14(self, cmd, val):
        self.send_command(cmd, val&0x7f, val>>7)
        
    def serialW7(self, val):
        self.port.write([val & 0x7F])
        
    def sendCommandHeader(self, cmd):
        if self.device_number == None:
            self.port.write([cmd])
        else:
            self.port.write([0xAA])
            self.serialW7(_deviceNumber)
            self.serialW7(cmd)
            
    def commandW7(self, cmd, val):
        self.sendCommandHeader(cmd)
        self.serialW7(val)
        _lastError = 0
        
    def commandWs14(self, cmd, val):
        self.sendCommandHeader(cmd)
        self.serialW7(val)
        self.serialW7(val >> 7)
        _lastError = 0
        
    def commandQuick(self, cmd):
        if self.device_number == None:
            header = [cmd]  # Compact protocol
        else:
            header = [0xAA, device_number, cmd & 0x7F]  # Pololu protocol
        self.port.write(header)

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
        #self.commandWs14(JrkG2Command.ForceDutyCycleTarget, dutyCycle);
        self.send_commandWs14(JrkG2Command.ForceDutyCycleTarget, dutyCycle)
        
    def stopMotor(self):
        self.commandQuick(JrkG2Command.MotorOff)
    
    # now ##############################################################################
        
    def forceDutyCycle(self, dutyCycle):
        if dutyCycle > 600:
            dutyCycle = 600
        if dutyCycle < -600:
            dutyCycle = -600
        self.send_commandWs14(JrkG2Command.ForceDutyCycle, dutyCycle)

    def get_variables(self, offset, length):
        self.send_command(0xE5, offset, length)
        result = self.port.read(length)
        if len(result) != length:
            raise RuntimeError("Expected to read {} bytes, got {}."
            .format(length, len(result)))
        return bytearray(result)

    def get_target(self):
        b = self.get_variables(0x02, 2)
        return b[0] + 256 * b[1]

    def get_feedback(self):
        b = self.get_variables(0x04, 2)
        return b[0] + 256 * b[1]

# you can run "jrk2cmd --cmd-port" to get the right name to use here.
# Linux USB example:  "/dev/ttyACM0"
# macOS USB example:  "/dev/cu.usbmodem001234562"
# Windows example:    "COM6"
port_name = "/dev/ttyACM0"
 
# Choose the baud rate (bits per second).  This does not matter if you are
# connecting to the Jrk over USB.  If you are connecting via the TX and RX
# lines, this should match the baud rate in the Jrk's serial settings.
baud_rate = 9600
 
# Change this to a number between 0 and 127 that matches the device number of 
# your Jrk if there are multiple serial devices on the line and you want to
# use the Pololu Protocol.
device_number = None
 
port = serial.Serial(port_name, baud_rate, timeout=0.1, write_timeout=0.1)
 
jrk = JrkG2Serial(port, device_number)
 
feedback = jrk.get_feedback()
print("Feedback is {}.".format(feedback))
 
target = jrk.get_target()
print("Target is {}.".format(target))
 
new_target = 2248 if target < 2048 else 1848
print("Setting target to {}.".format(new_target))
#jrk.setTarget(new_target)
#jrk.forceDutyCycle(100)
#time.sleep(1)
#print("stop")
#jrk.stopMotor()