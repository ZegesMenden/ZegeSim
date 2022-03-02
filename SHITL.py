# simulation libraries


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

with serial.Serial() as ser:
    ser.baudrate = 115200
    ser.port = 'COM4'
    ser.open()
    ser.timeout=0.5
ser.open()
while True:
    # print(ser.readline())
    if "a" in str(ser.readline()):
        print("header recieved")

        buf = []
        buf = ser.read(32)
        print(str(len(buf)))
        print(buf)
# def loop(rocket: rocketBody) -> control_data:
    
#     cd: control_data = control_data()
#     return cd