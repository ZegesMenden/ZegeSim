import numpy as np
import math
import random

from simulation.physics import *
from libs.controlMath import *

class barometer:

    def __init__(self) -> None:
        self.altitude = 0.0
        self.noise = 0.0
        self.lastRead = 0.0
        self.lastAltitude = 0.0
        self.lastVelocity = 0.0
        self.velocity = 0.0
        self.readDelay = 0.0
        self.pressureEventOffset = 0.0
        self.timeSincePressureEvent = 0.0

    def read(self, altitude, time) -> None:
        if time > self.lastRead + self.readDelay:
            if self.pressureEventOffset > 0:
                self.pressureEventOffset -= self.pressureEventOffset * \
                    0.5 * (time - self.lastRead)
            self.altitude = altitude + \
                ((random.randint(0, 100) / 100) *
                 self.noise * positive_or_negative())
            self.velocity = (self.altitude - self.lastAltitude) / \
                (time - self.lastRead)
            self.lastRead = time
            self.lastAltitude = self.altitude
            self.lastVelocity = self.velocity

    def pressureEvent(self, effect, time) -> None:
        self.pressureEventOffset = effect
        self.timeSincePressureEvent = time


class IMU6DOF:

    def __init__(self) -> None:
        """Class representing an IMU with sensor noise, bias, and sampling rate limiting"""

        self.accel = vector3(0.0, 0.0, 0.0)
        self.oriRates = vector3(0.0, 0.0, 0.0)

        self.gyroBias = vector3(0.0, 0.0, 0.0)
        self.accelBias = vector3(0.0, 0.0, 0.0)

        self.accelScale = vector3(0.0, 0.0, 0.0)
        self.gyroScale = vector3(0.0, 0.0, 0.0)

        self.gyroNoise = vector3(0.0, 0.0, 0.0)
        self.accelNoise = vector3(0.0, 0.0, 0.0)

        self.sampleRateAccel = 0.0
        self.sampleRateGyro = 0.0

        self.lastReadAccel = 0.0
        self.lastReadGyro = 0.0

    def readAccel(self, trueAccelerations, time) -> None:
        if time > self.lastReadAccel + self.sampleRateAccel:

            self.lastReadAccel = time

            self.accel.x = trueAccelerations.x + np.random.normal(0, self.accelNoise.x, 1)[0]
            self.accel.y = trueAccelerations.y + np.random.normal(0, self.accelNoise.y, 1)[0]
            self.accel.z = trueAccelerations.z + np.random.normal(0, self.accelNoise.z, 1)[0]

            self.accel += self.accelBias

            if self.accel.x > self.accelScale:
                self.accel.x = self.accelScale
            if self.accel.x < -self.accelScale:
                self.accel.x = -self.accelScale
            if self.accel.y > self.accelScale:
                self.accel.y = self.accelScale
            if self.accel.y < -self.accelScale:
                self.accel.y = -self.accelScale
            if self.accel.z > self.accelScale:
                self.accel.z = self.accelScale
            if self.accel.z < -self.accelScale:
                self.accel.z = -self.accelScale

    def readGyro(self, trueOriRates, time) -> None:

        if time > self.lastReadGyro + self.sampleRateGyro:

            self.lastReadGyro = time

            self.oriRates.x = trueOriRates.x + np.random.normal(0, self.gyroNoise.x, 1)[0] * DEG_TO_RAD
            self.oriRates.y = trueOriRates.y + np.random.normal(0, self.gyroNoise.y, 1)[0] * DEG_TO_RAD
            self.oriRates.z = trueOriRates.z + np.random.normal(0, self.gyroNoise.z, 1)[0] * DEG_TO_RAD

            self.oriRates += self.gyroBias

            if self.oriRates.x > self.gyroScale:
                self.oriRates.x = self.gyroScale
            if self.oriRates.x < -self.gyroScale:
                self.oriRates.x = -self.gyroScale

            if self.oriRates.y > self.gyroScale:
                self.oriRates.y = self.gyroScale
            if self.oriRates.y < -self.gyroScale:
                self.oriRates.y = -self.gyroScale

            if self.oriRates.z > self.gyroScale:
                self.oriRates.z = self.gyroScale
            if self.oriRates.z < -self.gyroScale:
                self.oriRates.z = -self.gyroScale


class GPS:

    def __init__(self) -> None:

        self.measuredPosition = vector3()
        self.measuredVelocity = vector3()
        self.signs = vector3(random.choice(
            [-1, 1]), random.choice([-1, 1]), random.choice([-1, 1]))
        self.lastRead = 0.0
        self.accuracy = 0.0

    def update(self, position, velocity, dt, time) -> None:

        self.accuracy += random.randint(0, 100) / 1000.0 * random.choice([-1, 1])

        if self.accuracy < 0.1:
            self.accuracy = 0.1

        if self.accuracy > 0.75:
            self.accuracy = 0.75

        self.measuredPosition = position + \
            (vector3(self.accuracy, self.accuracy, self.accuracy) * self.signs)
        self.measuredVelocity = velocity + \
            (vector3(self.accuracy, self.accuracy, self.accuracy) * self.signs * dt)

        self.lastRead = time
