from __future__ import annotations
from dataclasses import dataclass
import numpy as np

DEG_TO_RAD = np.pi / 180.0
RAD_TO_DEG = 180 / np.pi

# ty orlando <3


def clamp(n, minn, maxn):  # Clamps output to a range
    return max(min(maxn, n), minn)


@dataclass
class vector3:

    """Data class representing a 3-dimensional vector."""

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    def __init__(self, x: float = 0.0, y: float = 0.0, z: float = 0.0) -> None:
        """Initialize a vector3 object.

        parameters:

        x : float
            x-coordinate of the vector

        y : float
            y-coordinate of the vector

        z : float
            z-coordinate of the vector
        """

        self.x = x
        self.y = y
        self.z = z

        pass

    def __str__(self) -> str:
        """Return a string representation of the vector."""

        return f"{self.x}, {self.y}, {self.z}"

    def __repr__(self) -> str:
        """Return a complex string representation of the vector."""

        return f"vector3({self.x}, {self.y}, {self.z})"

    def __add__(self, other: vector3) -> vector3:
        """Add two vectors."""

        if isinstance(other, vector3):
            return vector3(self.x + other.x, self.y + other.y, self.z + other.z)
        else:
            return NotImplemented

    def __sub__(self, other: vector3) -> vector3:
        """Subtract two vectors."""

        if isinstance(other, vector3):
            return vector3(self.x - other.x, self.y - other.y, self.z - other.z)
        else:
            return NotImplemented

    def __mul__(self, other) -> vector3:
        """Multiply a vector by another vector or a scalar."""

        if isinstance(other, vector3):
            return vector3(self.x * other.x, self.y * other.y, self.z * other.z)
        if isinstance(other, float) or isinstance(other, int):
            return vector3(self.x * other, self.y * other, self.z * other)
        else:
            return NotImplemented

    def __truediv__(self, other) -> vector3:
        """Divide a vector by another vector or a scalar."""

        if isinstance(other, vector3):
            return vector3(self.x / other.x, self.y / other.y, self.z / other.z)
        elif isinstance(other, float) or isinstance(other, int):
            return vector3(self.x / other, self.y / other, self.z / other)
        else:
            return NotImplemented

    def __neg__(self) -> vector3:
        """Invert the sign of each vector's components."""

        return vector3(-self.x, -self.y, -self.z)

    def __pos__(self) -> vector3:
        """Make the sign of all components in the vector positive."""

        return vector3(+self.x, +self.y, +self.z)

    def __abs__(self) -> float:
        """Make the sign of all components in the vector positive."""

        return vector3(abs(self.x), abs(self.y), abs(self.z))

    def __eq__(self, other: vector3) -> bool:
        """Check if two vectors are equal."""

        if isinstance(other, vector3):
            return self.x == other.x and self.y == other.y and self.z == other.z
        else:
            return NotImplemented

    def __ne__(self, other: vector3) -> bool:
        """Check if two vectors are not equal."""

        if isinstance(other, vector3):
            return self.x != other.x or self.y != other.y or self.z != other.z
        else:
            return NotImplemented

    def norm(self) -> float:
        """The norm of the vector"""

        return np.sqrt(self.x**2 + self.y**2 + self.z**2)

    def normalize(self) -> vector3:
        """Normalize the vector."""

        return self / self.norm()

    def dot(self, other: vector3) -> float:
        """Dot product of two vectors."""

        if isinstance(other, vector3):
            return self.x * other.x + self.y * other.y + self.z * other.z
        else:
            return NotImplemented

    def cross(self, other: vector3) -> vector3:
        """Cross product of two vectors."""

        if isinstance(other, vector3):
            return vector3(self.y * other.z - self.z * other.y, self.z * other.x - self.x * other.z, self.x * other.y - self.y * other.x)
        else:
            return NotImplemented

    def angle_between_vectors(self, vector: vector3) -> float:
        """Calculate the angle between two vectors."""
        if isinstance(vector, vector3):
            return np.arccos(self.dot(vector) / (self.norm() * vector.norm()))
        else:
            return None

    def __round__(self, n) -> vector3:
        """Round the components of the vector to the nearest nth place."""

        return vector3(round(self.x, n), round(self.y, n), round(self.z, n))


@dataclass
class quaternion:

    """Class representing a quaternion."""

    w: float = 1.0
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    def __init__(self, w: float = 1.0, x: float = 0.0, y: float = 0.0, z: float = 0.0) -> None:
        """Initialize a quaternion object.

        parameters:

        w : float
            w-component of the quaternion

        x : float
            x-component of the quaternion

        y : float
            y-component of the quaternion

        z : float
            z-component of the quaternion
        """

        self.w = w
        self.x = x
        self.y = y
        self.z = z

        pass

    def __str__(self) -> str:
        """Return a string representation of the quaternion."""

        return f"{self.w}, {self.x}, {self.y}, {self.z}"

    def __repr__(self) -> str:
        """Return a complex string representation of the quaternion."""

        return f"quaternion({self.w}, {self.x}, {self.y}, {self.z})"

    def __mul__(self, other: quaternion) -> quaternion:
        """Multiply two quaternions.

        parameters:

        other : quaternion
            quaternion to multiply by
        """

        qNew: quaternion = quaternion(1.0, 0.0, 0.0, 0.0)

        qNew.w = (self.w * other.w) - (self.x * other.x) - (self.y *
                                                            other.y) - (self.z * other.z)  # im no betting man but if i were
        qNew.x = (self.w * other.x) + (self.x * other.w) + (self.y *
                                                            other.z) - (self.z * other.y)  # i would bet that at least one
        qNew.y = (self.w * other.y) - (self.x * other.z) + (self.y * other.w) + \
            (self.z * other.x)  # of the operations in this function
        qNew.z = (self.w * other.z) + (self.x * other.y) - \
            (self.y * other.x) + (self.z * other.w)  # is wrong

        # future ZegesMenden here - i was right

        return qNew

    def conj(self) -> quaternion:
        """Return the conjugate of the quaternion."""

        return quaternion(self.w, -self.x, -self.y, -self.z)

    def norm(self) -> float:
        """Return the norm of the quaternion."""

        return np.sqrt(self.w**2 + self.x**2 + self.y**2 + self.z**2)

    def normalize(self) -> quaternion:
        """Normalize the quaternion."""

        n: float = self.norm()
        return quaternion(self.w / n, self.x / n, self.y / n, self.z / n)

    def rotate(self, vector) -> vector3:
        """Rotate a vector by the quaternion.

        parameters:

        vector : vector3
            vector to rotate
        """
        if isinstance(vector, vector3):

            q = quaternion(0.0, vector.x, vector.y, vector.z)

            q = self * q * self.conj()

            return vector3(q.x, q.y, q.z)

        elif isinstance(vector, quaternion):

            q = self * vector * self.conj()

            return q
        else:
            return NotImplemented

    def fractional(self, a) -> quaternion:
        """Return the fractional of the quaternion."""

        self.w = 1-a + a*self.w
        self.x *= a
        self.y *= a
        self.z *= a

        return self.norm()

    def rotation_between_vectors(self, vector: vector3) -> quaternion:
        """Return the rotation quaternion between two vectors.

        parameters:

        vector : vector3
            vector to rotate
        """

        q = quaternion(0.0, vector.x, vector.y, vector.z)

        q = self * q
        q.w = 1 - q.w

        return q.normalize()

    def from_axis_angle(self, axis: vector3, angle: float) -> quaternion:
        """Return the quaternion from an axis and angle.

        parameters:

        axis : vector3
            axis of rotation

        angle : float
            angle of rotation
        """

        sa: float = np.sin(angle / 2)

        self.w = np.cos(angle / 2)
        self.x = axis.x * sa
        self.y = axis.y * sa
        self.z = axis.z * sa

        return self

    def euler_to_quaternion(self, euler_angles: vector3) -> quaternion:
        """Convert euler angles to a quaternion.

        parameters:

        euler_angles : vector3
            euler angles to convert
        """

        cr = np.cos(euler_angles.x / 2.0)
        cp = np.cos(euler_angles.y / 2.0)
        cy = np.cos(euler_angles.z / 2.0)

        sr = np.sin(euler_angles.x / 2.0)
        sp = np.sin(euler_angles.y / 2.0)
        sy = np.sin(euler_angles.z / 2.0)

        self.w = cr * cp * cy + sr * sp * sy
        self.x = sr * cp * cy - cr * sp * sy
        self.y = cr * sp * cy + sr * cp * sy
        self.z = cr * cp * sy - sr * sp * cy

        return self

    def quaternion_to_euler(self) -> vector3:
        """Convert a quaternion to euler angles."""

        # x = np.arctan2(2.0*self.y*self.w-2.0*self.x*self.z, 1.0-2.0*self.y*self.y-2.0*self.z*self.z)
        # y = np.arcsin(2.0*self.x*self.y+2.0*self.z*self.w)
        # z = np.arctan2(2.0*self.x*self.w-2.0*self.y*self.z, 1.0-2.0*self.x*self.x-2.0*self.z*self.z)

        x = np.arctan2(2.0 * (self.w * self.x + self.y * self.z),
                       1.0 - 2.0 * (self.x**2 + self.y**2))
        y = np.arcsin(2.0 * (self.w * self.y - self.z * self.x))
        z = np.arctan2(2.0 * (self.w * self.z + self.x * self.y),
                       1.0 - 2.0 * (self.y**2 + self.z**2))

        return vector3(x, y, z)

    def from_vector(self, vector: vector3) -> quaternion:
        """Return the quaternion from a vector.

        parameters:

        vector : vector3
            vector to convert
        """

        self.w = 0.0
        self.x = vector.x
        self.y = vector.y
        self.z = vector.z

        return self


class TVC:

    def __init__(self):

        self.command = vector3()

        self.position = vector3()

        self.servo_position = vector3()

        self.min = vector3()
        self.max = vector3()

        self.offset = vector3()

        self.noise = vector3()

        self.servo_speed = 0.0

        self.linkage_ratio = 0.0

        self.force = vector3()

    def actuate(self, command_angles: vector3, dt):

        self.command.y = command_angles.y * RAD_TO_DEG * self.linkage_ratio
        self.command.z = command_angles.z * RAD_TO_DEG * self.linkage_ratio
        
        actuation_y = clamp(
            self.command.y - self.servo_position.y, -self.servo_speed, self.servo_speed)
        actuation_z = clamp(
            self.command.z - self.servo_position.z, -self.servo_speed, self.servo_speed)
        
        self.servo_position.y += actuation_y * dt
        self.servo_position.z += actuation_z * dt

        self.servo_position.y = clamp(
            self.servo_position.y, self.min.y, self.max.y)
        self.servo_position.z = clamp(
            self.servo_position.z, self.min.z, self.max.z)

        self.position.y = ((self.servo_position.y +
                           self.offset.y) / self.linkage_ratio) * DEG_TO_RAD
        self.position.z = ((self.servo_position.z +
                           self.offset.z) / self.linkage_ratio) * DEG_TO_RAD

    def calculate_forces(self, thrust):

        self.force.y = np.sin(self.position.y) * thrust
        self.force.z = np.sin(self.position.z) * thrust
        self.force.x = thrust * \
            np.cos(self.position.y) - (thrust -
                                       (thrust * np.cos(self.position.z)))


@dataclass
class physics_body:

    """Class representing a rigid body in 3 dimensional space.

    parameters:

    mass : float
        mass of the body

    position : vector3
        position of the body

    velocity : vector3
        velocity of the body

    angular_velocity : vector3
        angular velocity of the body

    orientation : quaternion
        orientation of the body as a quaternion

    moment_of_inertia : vector3
        moment of inertia of the body

    """

    position: vector3 = vector3()
    velocity: vector3 = vector3()

    acceleration: vector3 = vector3()
    acceleration_local: vector3 = vector3()

    gravity: vector3 = vector3()

    rotation: quaternion = quaternion()
    rotation_euler: vector3 = vector3()
    rotational_velocity: vector3 = vector3()

    rotational_velocity_local: vector3 = vector3()

    rotational_acceleration: vector3 = vector3()

    mass: float = 1.0
    moment_of_inertia: vector3 = vector3()

    floor: bool = True

    wind: vector3 = vector3()
    drag_force: vector3 = vector3()
    drag_area: float = 0.0
    drag_coefficient: float = 0.0

    def __init__(self, position: vector3 = vector3(), velocity: vector3 = vector3(), acceleration: vector3 = vector3(), gravity: vector3 = vector3(),
                 mass: float = 0.0, moment_of_inertia: vector3 = vector3(), floor: bool = True, rotation: quaternion = quaternion(), rotational_velocity: vector3 = vector3(),
                 rotational_acceleration: vector3 = vector3(), acceleration_local: vector3() = vector3(), rotational_velocity_local: vector3() = vector3(), rotation_euler: vector3 = vector3(),
                 wind: vector3 = vector3(), drag_force: vector3 = vector3(), drag_area: float = 0.0, drag_coefficient: float = 0.0) -> None:
        """Initialize the physics body."""

        self.position = position
        self.velocity = velocity
        self.acceleration = acceleration
        self.gravity = gravity
        self.mass = mass
        self.moment_of_inertia = moment_of_inertia
        self.rotation = rotation
        self.rotational_velocity = rotational_velocity
        self.rotational_acceleration = rotational_acceleration
        self.floor = floor
        self.acceleration_local = acceleration_local
        self.rotational_velocity_local = rotational_velocity_local
        self.rotation_euler = rotation_euler

        self.wind = wind
        self.drag_force = drag_force
        self.drag_area = drag_area
        self.drag_coefficient = drag_coefficient

        pass

    def add_force(self, force: vector3) -> None:
        """Add a force to the body.

        parameters:

        force : vector3
            force to add
        """

        self.acceleration += force / self.mass

    def add_torque(self, torque: vector3) -> None:
        """Add a torque to the body.

        parameters:

        torque : vector3
            torque to add
        """
        
        self.rotational_acceleration += torque / self.moment_of_inertia

    def add_force_local(self, force: vector3) -> None:
        """Add a force to the body in local space.

        parameters:

        force : vector3
            force to add
        """

        nf: vector3 = self.rotation.rotate(force)

        self.add_force(nf)

    def add_torque_local(self, torque: vector3) -> None:
        """Add a torque to the body in local space.

        parameters:

        torque : vector3
            torque to add
        """

        nt: vector3 = self.rotation.rotate(torque)

        self.add_torque(nt)

    def add_global_point_force(self, force: vector3, point: vector3) -> None:
        """Add a force to the body at a point in global space.

        parameters:

        point : vector3
            point to apply the force

        force : vector3
            force to apply
        """

        torque: vector3 = force * point.x
        torque.x = 0.0
        self.add_torque(torque)

        self.add_force(force)

    def add_local_point_force(self, force: vector3, point: vector3) -> None:
        """Add a force to the body at a point.

        parameters:

        force : vector3
            force to add

        point : vector3
            point to add the force to
        """
        if isinstance(point, vector3) and isinstance(force, vector3):

            nf: vector3 = self.rotation.rotate(force)
            np: vector3 = self.rotation.rotate(point)

            self.add_global_point_force(nf, point)

    def update_aero(self):

        """Updates aerodynamic forces acting on the body.
        
        Note - you still need to apply the drag force to the physics body with apply_glocal_point_force()."""

        velocity_relative_wind: vector3 = self.velocity - self.wind_speed

        if velocity_relative_wind.x != 0.0 and velocity_relative_wind.y != 0.0 and velocity_relative_wind.z != 0.0:

            self.aoa = velocity_relative_wind.angle_between_vectors(
                self.rotation.rotate(vector3(1.0, 0.0, 0.0)))

            dc = self.drag_coefficient * self.aoa

            if self.floor == True and self.position.x != 0.0:
                self.drag_force = -velocity_relative_wind.normalize() * 0.5 * 1.225 * \
                    (self.velocity.norm() ** 2) * dc * self.drag_area
            elif self.floor == False:
                self.drag_force = -velocity_relative_wind.normalize() * 0.5 * 1.225 * \
                    (self.velocity.norm() ** 2) * dc * self.drag_area

    def update(self, dt: float) -> None:
        """Updates the physics body"""

        self.acceleration_local = self.rotation.conj().rotate(self.acceleration)

        self.acceleration += self.gravity

        self.velocity += self.acceleration * dt
        self.position += self.velocity * dt

        self.rotational_velocity += self.rotational_acceleration * dt

        ang = self.rotational_velocity.norm()

        if ang == 0.0:
            ang = 0.000000001

        self.rotation *= quaternion(0.0, 0.0, 0.0, 0.0).from_axis_angle(self.rotational_velocity / ang, ang * dt)

        self.rotation_euler = self.rotation.quaternion_to_euler()

        self.rotational_velocity_local = self.rotation.conj().rotate(self.rotational_velocity)

        # self.rotation_euler.x = 0.0
        # self.rotational_velocity.x = 0.0
        # self.rotation = quaternion().euler_to_quaternion(self.rotation_euler)

        if self.floor and self.position.x <= 0.0:

            self.position.x = 0.0
            self.velocity.x = 0.0

    def clear(self) -> None:
        """clears the rotational and translational acceleration"""

        self.acceleration = vector3(0.0, 0.0, 0.0)
        self.rotational_acceleration = vector3(0.0, 0.0, 0.0)
