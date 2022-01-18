import numpy
from simulation.physics import *
from libs.controlMath import *
from simulation.motors import * 

class nav:

    def __init__(self) -> None:

        self.accelBias = vector3(0.0, 0.0, 0.0)

        self.gyroscopeBias = vector3(0.0, 0.0, 0.0)

        self.barometerAlt = 0.0
        self.barometerVel = 0.0
        self.barometerTime = 0.0

        self.oriRates = vector3(0.0, 0.0, 0.0)
        self.lastOriRates = vector3(0.0, 0.0, 0.0)

        self.orientation_quat: quaternion = quaternion()
        self.orientation_euler = vector3(0.0, 0.0, 0.0)

        self.accelerationLocal = vector3(0.0, 0.0, 0.0)
        self.lastAccelerationLocal = vector3(0.0, 0.0, 0.0)

        self.accelerationInertial = vector3(0.0, 0.0, 0.0)
        
        self.velocityInertial = vector3(0.0, 0.0, 0.0)
        self.positionInertial = vector3(0.0, 0.0, 0.0)

        self.debiasCount = 0

        self.debiased = False
        self.inFlight = False

        self.posKF_x = kalman()
        self.posKF_y = kalman()
        self.posKF_z = kalman()

    def update(self, acceleration, rotationalVel, gravity, dt) -> None:

        self.accelerationLocal = acceleration - self.accelBias
        self.oriRates = rotationalVel - self.gyroscopeBias
        
        if self.inFlight == False:
            self.positionInertial = vector3(0, 0 ,0)
            self.velocityInertial = vector3(0, 0, 0)
        
        ang = self.oriRates.norm()

        self.orientation_quat *= quaternion().from_axis_angle(self.oriRates / ang, ang * dt)

        self.orientation_euler = self.orientation_quat.quaternion_to_euler()
        
        # self.orientation_euler.x = 0.0
        # self.orientation_quat = quaternion().euler_to_quaternion(self.orientation_euler)

        self.accelerationInertial = self.orientation_quat.rotate(self.accelerationLocal)

        self.accelerationInertial += gravity

        self.posKF_x.update_accel(self.accelerationInertial.x, dt)
        self.posKF_y.update_accel(self.accelerationInertial.y, dt)
        self.posKF_z.update_accel(self.accelerationInertial.z, dt)

        self.velocityInertial.x = self.posKF_x.vel
        self.velocityInertial.y = self.posKF_y.vel
        self.velocityInertial.z = self.posKF_z.vel

        self.positionInertial.x = self.posKF_x.pos
        self.positionInertial.y = self.posKF_y.pos
        self.positionInertial.z = self.posKF_z.pos

        if self.accelerationLocal.x > 2:
            self.inFlight = True

        if self.positionInertial.x <= 0:
            self.positionInertial.x = 0
            self.velocityInertial.x = 0
        # else:
        #     self.accelerationLocalFiltered = self.accelerationLocal
        #     self.oriRatesFiltered = self.oriRates
        
        self.last_acceleration = self.accelerationLocal
    
    def accelOri(self, accel) -> None:
        q = quaternion().from_vector(self.orientation_quat.rotate(accel)) * quaternion(0, 1, 0, 0)
        q.w = 1 - q.w
        self.orientation_quat = quaternion().from_vector(self.orientation_quat.conj().rotate(q.fractional(0.5)))

    def passBarometerData(self, barometerAlt, barometerVel, time) -> None:
        self.barometerAlt = barometerAlt
        self.barometerVel = barometerVel
        self.barometerTime = time
        self.posKF_x.update_position(self.barometerAlt)

    def passGPSData(self, gpsData) -> None:
        
        if isinstance(gpsData, vector3):
            # self.posKF_x.update_position(gpsData.x)
            self.posKF_y.update_position(gpsData.y)
            self.posKF_z.update_position(gpsData.z)
    
    def measureDebias(self, acceleration, rotationalVel) -> None:
        if self.inFlight == False and self.debiased == False:
            self.debiasCount += 1
            self.accelBias += acceleration
            self.gyroscopeBias += rotationalVel

    def debias(self) -> None:
        if self.debiased == False and self.debiasCount > 0:
            self.accelBias /= self.debiasCount
            self.gyroscopeBias /= self.debiasCount
            self.debiased = True