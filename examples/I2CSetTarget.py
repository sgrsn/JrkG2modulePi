from smbus2 import SMBus, i2c_msg
from enum import IntEnum
import ctypes
from JrkG2modulePi import JrkG2

# you can run "i2cdetect -l" to get the right number to use here.
bus = SMBus(1)
# and you can run "i2cdetect -y 1"
# you can find the right address to use here.
# Normaly, this address is 0x0B
address = 0x0B
jrk = JrkG2.JrkG2I2C(bus, address)

feedback = jrk.getFeedback()
print("feedback : ", feedback)
target = jrk.getTarget()
print("target : ", target)
new_target = 2248 if target < 2048 else 1848
print("new target : ", new_target)
jrk.setTarget(new_target)
