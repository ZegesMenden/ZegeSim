# simulation libraries

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
    ser.baudrate = 19200
    ser.port = 'COM4'
    ser.open()
    ser.write(b'hello')
    
def setup(rocket: rocketBody):
    pass

def loop(rocket: rocketBody) -> control_data:
    
    cd: control_data = control_data()
    return cd