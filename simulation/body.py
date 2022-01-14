from simulation.physics import *
from simulation.motors import *
from simulation.sensors import *


class rocketBody:

    """Class representing a rocket"""

    def __init__(self) -> None:
        """initializes the rocket interface"""

        self.body: physics_body = physics_body()

        self.time: float = 0.0
        self.time_step: float = 0.0

        self.IMU: IMU6DOF = IMU6DOF()
        self.barometer: barometer = barometer()
        self.gps: GPS = GPS()

        self.tvc: TVC = TVC()
        self.tvc_position: vector3 = vector3()
        self.tvc_location : vector3 = vector3()
        self.reaction_wheel_torque: float = 0.0

        self.rocket_motor: rocketMotor = rocketMotor(0)

        self.tvc: TVC = TVC()

    def getTimeSeconds(self) -> float:
        return self.time

    def getTimeMilliseconds(self) -> float:
        return self.time * 1000

    def getTimeMicroseconds(self) -> float:
        return self.time * 1000000

    def update(self):

        self.rocket_motor.update(self.time)
        self.tvc.actuate(self.tvc_position, self.time_step)

        self.tvc.calculate_forces(self.rocket_motor.currentThrust)
        self.body.add_local_point_force(self.tvc.force, self.tvc_location)

        # self.body.update_aerodynamics()
        # self.body.applyForce(self.body.drag_force, self.body.cp_location)
        self.body.update(self.time_step)

    def clear(self):
        self.body.clear()
