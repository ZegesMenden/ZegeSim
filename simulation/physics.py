from dataclasses import dataclass
import numpy as np

@dataclass
class vector3:

    """Data class representing a 3-dimensional vector."""

    x : float = 0.0
    y : float = 0.0
    z : float = 0.0    

    def __init__(self, x : float = 0.0, y : float = 0.0, z : float = 0.0):
        
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

    def __str__(self):
        
        """Return a string representation of the vector."""

        return f"{self.x}, {self.y}, {self.z}"

    def __repr__(self):

        """Return a complex string representation of the vector."""

        return f"vector3({self.x}, {self.y}, {self.z})"

    def __add__(self, other : vector3):

        """Add two vectors."""

        if isinstance(other, vector3):
            return vector3(self.x + other.x, self.y + other.y, self.z + other.z)
        else:
            return NotImplemented

    def __sub__(self, other : vector3):

        """Subtract two vectors."""

        if isinstance(other, vector3):
            return vector3(self.x - other.x, self.y - other.y, self.z - other.z)
        else:
            return NotImplemented
    
    def __mul__(self, other : float):

        """Multiply a vector by a scalar."""

        if isinstance(other, float):
            return vector3(self.x * other, self.y * other, self.z * other)
        else:
            return NotImplemented

    def __mul__(self, other : vector3):

        """Multiply two vectors."""

        if isinstance(other, vector3):
            return vector3(self.x * other.x, self.y * other.y, self.z * other.z)
        else:
            return NotImplemented

    def __truediv__(self, other : float):

        """Divide a vector by a scalar."""

        if isinstance(other, float):
            return vector3(self.x / other, self.y / other, self.z / other)
        else:
            return NotImplemented

    def __truediv__(self, other : vector3):

        """Divide two vectors."""

        if isinstance(other, vector3):
            return vector3(self.x / other.x, self.y / other.y, self.z / other.z)
        else:
            return NotImplemented

    def __neg__(self):
        
        """Invert the sign of each vector's components."""

        return vector3(-self.x, -self.y, -self.z)
    
    def __pos__(self):
        
        """Make the sign of all components in the vector positive."""

        return vector3(+self.x, +self.y, +self.z)

    def __abs__(self):

        """Make the sign of all components in the vector positive."""
        
        return +self

    def __eq__(self, other : vector3):

        """Check if two vectors are equal."""

        if isinstance(other, vector3):
            return self.x == other.x and self.y == other.y and self.z == other.z
        else:
            return NotImplemented

    def __ne__(self, other : vector3):

        """Check if two vectors are not equal."""

        if isinstance(other, vector3):
            return self.x != other.x or self.y != other.y or self.z != other.z
        else:
            return NotImplemented

    def norm(self):
            
        """The norm of the vector"""

        return np.sqrt(self.x**2 + self.y**2 + self.z**2)

    def normalize(self):

        """Normalize the vector."""

        return self / self.norm()

    def dot(self, other : vector3):

        """Dot product of two vectors."""

        if isinstance(other, vector3):
            return self.x * other.x + self.y * other.y + self.z * other.z
        else:
            return NotImplemented

    def cross(self, other : vector3):

        """Cross product of two vectors."""

        if isinstance(other, vector3):
            return vector3(self.y * other.z - self.z * other.y, self.z * other.x - self.x * other.z, self.x * other.y - self.y * other.x)
        else:
            return NotImplemented

    def __round__(self, n):
    
        """Round the components of the vector to the nearest nth place."""

        return vector3(round(self.x, n), round(self.y, n), round(self.z, n))

@dataclass
class quaternion:

    """Class representing a quaternion."""

    w : float = 1.0
    x : float = 0.0
    y : float = 0.0
    z : float = 0.0

    def __init__(self, w : float = 1.0, x : float = 0.0, y : float = 0.0, z : float = 0.0):

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

    def __str__(self):
        
        """Return a string representation of the quaternion."""

        return f"{self.w}, {self.x}, {self.y}, {self.z}"
    
    def __repr__(self):
            
        """Return a complex string representation of the quaternion."""

        return f"quaternion({self.w}, {self.x}, {self.y}, {self.z})"

    def __mul__(self, other : quaternion):

        """Multiply two quaternions.

        parameters:

        other : quaternion
            quaternion to multiply by
        """

        qNew = quaternion(1.0, 0.0, 0.0, 0.0)

        qNew.w = (self.w * quaternion.w) - (self.x * quaternion.x) - (self.y * quaternion.y) - (self.z * quaternion.z)  # im no betting man but if i were
        qNew.x = (self.w * quaternion.x) + (self.x * quaternion.w) + (self.y * quaternion.z) - (self.z * quaternion.y)  # i would bet that at least one
        qNew.y = (self.w * quaternion.y) - (self.x * quaternion.z) + (self.y * quaternion.w) + (self.z * quaternion.x)  # of the operations in this function
        qNew.z = (self.w * quaternion.z) + (self.x * quaternion.y) - (self.y * quaternion.x) + (self.z * quaternion.w)  # is wrong

        # future ZegesMenden here - i was right

        return qNew

    def conj(self):

        """Return the conjugate of the quaternion."""

        return quaternion(self.w, -self.x, -self.y, -self.z)

    def norm(self):
        
        """Return the norm of the quaternion."""

        return np.sqrt(self.w**2 + self.x**2 + self.y**2 + self.z**2)

    def normalize(self):

        """Normalize the quaternion."""

        return self / self.norm()

    def rotate(self, vector : vector3):

        """Rotate a vector by the quaternion.

        parameters:

        vector : vector3
            vector to rotate
        """

        q = quaternion(0.0, vector.x, vector.y, vector.z)

        q = self * q * self.conj()

        return vector3(q.x, q.y, q.z)

    def rotate(self, vector : quaternion):

        """Rotate a vector by the quaternion.

        parameters:

        vector : quaternion
            vector to rotate
        """

        q = self * vector * self.conj()

        return q

    def fractional(self, a):

        """Return the fractional of the quaternion."""

        self.w = 1-a + a*self.w
        self.x *= a
        self.y *= a
        self.z *= a

        return self.norm()

    def rotation_between_vectors(self, vector : vector3):
            
        """Return the rotation quaternion between two vectors.

        parameters:

        vector : vector3
            vector to rotate
        """

        q = quaternion(0.0, vector.x, vector.y, vector.z)

        q = self * q
        q.w = 1 - q.w

        return q.normalize()

    def from_axis_angle(self, axis : vector3, angle : float):
    
        """Return the quaternion from an axis and angle.

        parameters:

        axis : vector3
            axis of rotation

        angle : float
            angle of rotation
        """

        sa : float = np.sin(angle / 2)

        self.w = np.cos(angle / 2)
        self.x = axis.x * sa
        self.y = axis.y * sa
        self.z = axis.z * sa

        return self

    def euler_to_quaternion(self, euler_angles : vector3):

        """Convert euler angles to a quaternion.

        parameters:

        euler_angles : vector3
            euler angles to convert
        """

        cr = np.cos(euler_angles.x / 2)
        cp = np.cos(euler_angles.y / 2)
        cy = np.cos(euler_angles.z / 2)

        sr = np.sin(euler_angles.x / 2)
        sp = np.sin(euler_angles.y / 2)
        sy = np.sin(euler_angles.z / 2)

        self.w = cr * cp * cy + sr * sp * sy
        self.x = sr * cp * cy - cr * sp * sy
        self.y = cr * sp * cy + sr * cp * sy
        self.z = cr * cp * sy - sr * sp * cy

        return self
    
    def quaternion_to_euler(self):

        """Convert a quaternion to euler angles."""

        x = np.arctan2(2 * (self.w * self.x + self.y * self.z), 1 - 2 * (self.x**2 + self.y**2))
        y = np.arcsin(2 * (self.w * self.y - self.z * self.x))
        z = np.arctan2(2 * (self.w * self.z + self.x * self.y), 1 - 2 * (self.y**2 + self.z**2))

        return vector3(x, y, z)

