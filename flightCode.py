from simulation.physics import *
from simulation.dataManagement import *
from simulation.controlMath import *
from simulation.motors import *
import simulation.sim as sim

import math
import random


def setup() -> None:
    # put your setup code here, to run once:

    pass


def loop() -> sim.control_data:
    # put your main code here, to run repeatedly:

    # must pass control data to the simulation
    return sim.control_data()
