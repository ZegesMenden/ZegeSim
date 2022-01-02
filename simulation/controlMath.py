import math
from typing import AsyncContextManager
import numpy as np
import random
from physics import *


def rotate(x, y, theta) -> vector3:
    cs = math.cos(theta)
    sn = math.sin(theta)

    rotated_x = x * cs - y * sn
    rotated_y = x * sn + y * cs

    return vector3(rotated_x, rotated_y, 0)
# thanks orlando again


def calculateAngleFromDesiredTorque(moment_arm, force, mmoi, desired_torque):
    calcval = desired_torque * mmoi / force / moment_arm
    return math.asin(calcval)


def positive_or_negative():
    if random.random() < 0.5:
        return 1
    else:
        return -1


def LPF(new, old, gain):
    return new*(1-gain) + old*gain


class PID:
    def __init__(self, kP, kI, kD, setpoint, iMax, usePONM):

        # assigning gains to user imathuts
        self.kP = kP
        self.kI = kI
        self.kD = kD

        # the value that the PID controller wants the imathut to become
        self.setPoint = setpoint

        # you might not see this in your run-of-the-mill PID library, this is just a limit on how bit the integral can get
        self.iMax = iMax

        # you are even less likely to see this in your run-of-the-mill PID library
        # this is basically a different implementation of the proportional gain, where instead of looking at the error
        # the proportional part of the controller looks at the change between the initial sensed value and the current sensed value

        self.usePONM = usePONM

        self.proportional = 0

        # this variable stores the most recent process variable, so that when update is called the PID controller can
        # know not only the current error, but also the speed at which it is approaching or departing from the setpoint!
        # this helps with reducing overshoot by feeding it into the derivitive value, which basically functions as a break on a car
        self.lastProcess = 0

        # this is the integral, which helps combat steady state error. Eg. your rocket is stable, and has canceled all rotation on
        # it's body, but its 10 degrees off target! the integral is crucial here as it increases over time with the error
        self.integral = 0
        self.error = 0

        self.derivitive = 0
        # for dataloging purpouses

        self.output = 0

    def setSetpoint(self, setpoint):
        self.setPoint = setpoint

    def zeroIntegral(self):
        self.integral = 0

    def compute(self, process, dt):

        change = process - self.lastProcess

        self.lastProcess = process

        self.error = self.setPoint - process

        if self.usePONM == True:
            # whats happening here is the proportional changing with the process variable
            self.proportional -= change * self.kP
        else:
            self.proportional = self.error * self.kP

        # checking that the integral will not exceed the maximum value allowed by the user
        if abs(self.integral + self.error * self.kI * dt) < self.iMax:
            self.integral += self.error * self.kI * dt

        self.derivitive = change / dt * self.kD

        if self.usePONM == True:
            self.output = self.proportional + self.integral - self.derivitive
        else:
            self.output = self.proportional + self.integral - self.derivitive


class FSF:

    def __init__(self, gain_a, gain_b):
        self.gain_a = gain_a
        self.gain_b = gain_b
        self.setpoint = 0.0
        self.out = 0.0
        self.lastOri = 0.0

    def compute(self, ori, oriRate):
        self.output = -self.gain_a * (ori - self.setpoint)
        self.output += -self.gain_b * oriRate
        self.lastOri = ori

    def getOutput(self):
        return self.output


class kalman:

    def __init__(self) -> None:

        self.P = [0.0, 0.0, 0.0, 0.0]
        self.vel = 0.0
        self.pos = 0.0

        self.Q = 0.0
        self.R = 0.0

    def update_accel(self, accel, dt) -> None:

        self.vel += accel * dt
        self.pos += self.vel * dt
        q_dtdt = self.Q * (dt * dt)
        self.P[0] += (self.P[2] + self.P[1] +
                      (self.P[3] + q_dtdt * 0.25) * dt) * dt
        self.P[1] += (self.P[3] + q_dtdt * 0.5) * dt
        self.P[2] = self.P[1]
        self.P[3] += q_dtdt

    def update_position(self, pos) -> None:

        y = self.pos - pos

        inv = 1.0 / (self.P[0] + self.R)

        K_a = self.P[0] * inv
        K_b = self.P[2] * inv

        self.pos += K_a * y
        self.vel += K_b * y

        self.vel += y
        self.vel /= 2.0

        self.pos += pos
        self.pos /= 2.0

        self.P[0] -= K_a * self.P[0]
        self.P[1] -= K_a * self.P[1]
        self.P[2] -= K_b * self.P[0]
        self.P[3] -= K_b * self.P[1]

    def get_position(self) -> float:

        return self.pos

    def get_velocity(self) -> float:

        return self.vel


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

            self.accel.x = np.random.normal(
                trueAccelerations.x, self.accelNoise.x, 1)[0]
            self.accel.y = np.random.normal(
                trueAccelerations.y, self.accelNoise.y, 1)[0]
            self.accel.z = np.random.normal(
                trueAccelerations.z, self.accelNoise.z, 1)[0]

            self.accel += self.accelBias

            if self.accel.x > self.accelScale.x:
                self.accel.x = self.accelScale.x
            if self.accel.x < -self.accelScale.x:
                self.accel.x = -self.accelScale.x
            if self.accel.y > self.accelScale.y:
                self.accel.y = self.accelScale.y
            if self.accel.y < -self.accelScale.y:
                self.accel.y = -self.accelScale.y
            if self.accel.z > self.accelScale.z:
                self.accel.z = self.accelScale.z
            if self.accel.z < -self.accelScale.z:
                self.accel.z = -self.accelScale.z

    def readGyro(self, trueOriRates, time) -> None:

        if time > self.lastReadGyro + self.sampleRateGyro:

            self.lastReadGyro = time

            self.oriRates.x = trueOriRates.x + \
                np.random.normal(0, self.gyroNoise.x, 1)[0] * DEG_TO_RAD
            self.oriRates.y = trueOriRates.y + \
                np.random.normal(0, self.gyroNoise.y, 1)[0] * DEG_TO_RAD
            self.oriRates.z = trueOriRates.z + \
                np.random.normal(0, self.gyroNoise.z, 1)[0] * DEG_TO_RAD

            self.oriRates += self.gyroBias

            if self.oriRates.x > self.gyroScale.x:
                self.oriRates.x = self.gyroScale.x
            if self.oriRates.x < -self.gyroScale.x:
                self.oriRates.x = -self.gyroScale.x

            if self.oriRates.y > self.gyroScale.y:
                self.oriRates.y = self.gyroScale.y
            if self.oriRates.y < -self.gyroScale.y:
                self.oriRates.y = -self.gyroScale.y

            if self.oriRates.z > self.gyroScale.z:
                self.oriRates.z = self.gyroScale.z
            if self.oriRates.z < -self.gyroScale.z:
                self.oriRates.z = -self.gyroScale.z


class GPS:

    def __init__(self) -> None:

        self.measuredPosition = vector3()
        self.measuredVelocity = vector3()
        self.signs = vector3(random.choice(
            [-1, 1]), random.choice([-1, 1]), random.choice([-1, 1]))
        self.lastRead = 0.0
        self.accuracy = 0.0

    def update(self, position, velocity, dt, time) -> None:

        self.accuracy += random.randint(0, 100) / \
            1000.0 * random.choice([-1, 1])

        if self.accuracy < 0.1:
            self.accuracy = 0.1

        if self.accuracy > 0.75:
            self.accuracy = 0.75

        self.measuredPosition = position + \
            (vector3(self.accuracy, self.accuracy, self.accuracy) * self.signs)
        self.measuredVelocity = velocity + \
            (vector3(self.accuracy, self.accuracy, self.accuracy) * self.signs * dt)

        self.lastRead = time
