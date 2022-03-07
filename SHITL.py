# simulation libraries
import msvcrt

from matplotlib.style import available
from simulation.physics import *
from simulation.dataManagement import *
from simulation.motors import *
from simulation.body import *

# user libraries

from libs.controlMath import *
from libs.ori import *

import numpy as np
import random

import time
from datetime import datetime

import serial

com_start = False

with serial.Serial() as ser:
    ser.baudrate = 115200
    ser.port = 'COM6'
    ser.timeout=0.5

def init_comms():

    print("SITL: initializing communications with callisto")

    ser.open()

    print("SITL: communications established")

print("SITL: initializing")
print("as if there's anything to initialize lmfao") 
print("entering main loop, type help for a list of the commands")
while True:
    if com_start:
        inString = ser.readline()
        if inString:
            if chr(inString[0]) != '#':
                print(str(inString)[2:-5])
            # elif chr(inString[0]) == '#':
                

    if msvcrt.kbhit():
        inp = input()
        if inp.upper() == 'HELP':
            print('''SITL: HELP:
exit - exits the program
help - prints this message
run - runs the simulation
init - initializes communications with callisto
close - stops communications with callisto and closes the serial port
print <data> - prints data to callisto''')
        elif inp.upper() == 'EXIT':
            ser.close()
            print("SITL: exiting")
            break
        elif inp.upper() == 'RUN':
            if not com_start:
                print(f"SITL: initializing communication with callisto on {ser.port}")
                com_start = True
                init_comms()
        elif inp.upper() == 'INIT':
            if not com_start:
                print("SITL: initializing communications with callisto")
                com_start = True
                init_comms()
            else:
                print("SITL: communications already established")
        elif inp.upper() == 'CLOSE':
            if com_start:
                print("SITL: closing communications with callisto")
                com_start = False
                ser.close()
        elif 'PRINT' in inp.upper():
            if com_start:
                print(f"SITL: printing {inp[6:]}")
                ser.write(inp[6:].encode())
            else:
                print("SITL: communications not established")
        else:
            print(f"SITL: WHAT THE FUCK IS {inp} SUPPSED TO MEAN!?")

0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0

# def loop(rocket: rocketBody) -> control_data:
    
#     cd: control_data = control_data()
#     return cd