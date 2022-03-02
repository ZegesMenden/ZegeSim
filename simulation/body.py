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

        self.rocket_motor: rocket_motor = rocket_motor(1000)
        self.cp_locaoation: vector3 = vector3()
        self.dry_mass: float = 1.0

        self.tvc: TVC = TVC()

    def getTimeSeconds(self) -> float:
        return self.time

    def getTimeMilliseconds(self) -> float:
        return self.time * 1000

    def getTimeMicroseconds(self) -> float:
        return self.time * 1000000

    def update(self):
        
        self.body.mass = self.dry_mass + (self.rocket_motor.current_mass*0.001)

        self.rocket_motor.update(self.time)
        self.tvc.actuate(self.tvc_position, self.time_step)

        self.tvc.calculate_forces(self.rocket_motor.current_thrust)
        self.body.add_force_local(self.tvc.force)
        self.body.add_torque_local(vector3(0.0, self.tvc.force.y, self.tvc.force.z) * self.tvc_location.x)
        
        self.body.update_aero()
        self.body.add_force(self.body.drag_force)
        self.body.add_torque(self.cp_locaoation.cross(self.body.drag_force))
        self.body.add_torque(self.body.rotational_velocity * -0.001)
        self.body.update(self.time_step)
        
        # self.body.rotation_euler *= 0.95
        # self.body.rotation = self.body.rotation.euler_to_quaternion(self.body.rotation_euler)

    def clear(self):
        self.body.clear()
