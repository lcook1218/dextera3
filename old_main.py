import sys
import readline

from arm import Arm

dextera = Arm()
while True:
    command = input ('command: ')
    dextera.parse_text(command)

#when program is stopped, the motor does not stop