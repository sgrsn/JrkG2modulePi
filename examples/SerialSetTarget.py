import serial
from enum import IntEnum
import ctypes
from JrkG2modulePi import JrkG2

# you can run "jrk2cmd --cmd-port" to get the right name to use here.
# Linux USB example:  "/dev/ttyACM0"
# macOS USB example:  "/dev/cu.usbmodem001234562"
# Windows example:    "COM6"
port_name = "/dev/ttyACM0"
baud_rate = 9600
device_number = None

port = serial.Serial(port_name, baud_rate, timeout=0.1, write_timeout=0.1)
jrk = JrkG2.JrkG2Serial(port, device_number)
 
feedback = jrk.getFeedback()
print("feedback : ", feedback)
target = jrk.getTarget()
print("target : ", target)
new_target = 2248 if target < 2048 else 1848
print("new target : ", new_target)
jrk.setTarget(new_target)
