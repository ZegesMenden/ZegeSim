# simulation libraries
from simulation.physics import *
from simulation.dataManagement import *
from simulation.motors import *

# user libraries

from libs.controlMath import *

import math
import random


def setup() -> bool:
    # put your setup code here, to run once:

    return True


def loop() -> control_data:
    # put your main code here, to run repeatedly:

    # must pass control data to the simulation
    return control_data()
