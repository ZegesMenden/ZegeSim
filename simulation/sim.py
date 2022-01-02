from dataclasses import dataclass

from physics import *
from dataManagement import *
from controlMath import *
from motors import *

import flightCode


@dataclass
class control_data:

    """Class storing all the control outputs of the rocket

    Params:

    tvc_position - a vector represenging the position of the rocket's thrust vector control mount in radians

    reaction_wheel_output - a torque in newton-meters representing the torque from a reaction wheel

    motor_fire - the name of the motor you want to fire, leave blank to not fire any motor
    """

    tvc_position: vector3 = vector3()
    reaction_wheel_output: float = 0.0

    motor_fire: str = ""

dataIn: control_data = control_data()

class rocketBody:

    """Class representing a rocket"""

    def __init__(self) -> None:
        """initializes the rocket interface"""

        self.body: physicsBody = physicsBody()

        self.time: float = 0.0

        self.IMU: IMU6DOF = IMU6DOF()
        self.barometer: barometer = barometer()
        self.gps: GPS = GPS()

        self.tvc_position: vector3 = vector3()
        self.reaction_wheel_torque: float = 0.0

        self.pyro1: bool = False
        self.pyro2: bool = False
        self.pyro3: bool = False

        self.rocket_motor: rocketMotor = rocketMotor(0)

        self.tvc : TVC = TVC()


rocket: rocketBody = rocketBody()

# load settings

settingsLoader = settingsParser()

settingsLoader.load_settings("../settings.yaml")

simulation_time: float = settingsLoader.simulation_time
simulation_time_step: float = settingsLoader.time_step
rocket.rocket_motor.timeStep = settingsLoader.time_step

rocket.body.dryMass = settingsLoader.mass
rocket.body.moment_of_inertia = settingsLoader.mmoi

rocket.body.drag_area = settingsLoader.drag_area
rocket.body.drag_coefficient = settingsLoader.drag_coeff
rocket.body.wind_speed = settingsLoader.wind_speed

rocket.IMU.sampleRateAccel = settingsLoader.imu_accel_read_speed
rocket.IMU.sampleRateGyro = settingsLoader.imu_gyro_read_speed

rocket.barometer.readDelay = settingsLoader.baro_read_speed
rocket.rocket_motor.maxIgnitionDelay = settingsLoader.max_ignition_delay

rocket.tvc

for motor in settingsLoader.motors:
    print(motor[0], motor[1])
    rocket.rocket_motor.add_motor(motor[1], motor[0])

rocket.body.mass = rocket.body.dryMass + rocket.rocket_motor.totalMotorMass


def getTimeSeconds() -> float:
    return simulation_time


def getTimeMilliseconds() -> float:
    return simulation_time * 1000


def getTimeMicroseconds() -> float:
    return simulation_time * 1000000

# simulation


flightCode.setup()

while True:

    simulation_time += simulation_time_step

    dataIn = flightCode.loop()

    rocket.rocket_motor.ignite(dataIn.motor_fire, simulation_time)

    rocket.body.update(simulation_time_step)

    if rocket.time > simulation_time:
        break
