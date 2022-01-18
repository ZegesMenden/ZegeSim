import math
from typing import AsyncContextManager
import numpy as np
import random
from simulation.physics import *


def rotate(x, y, theta) -> vector3:
    cs = math.cos(theta)
    sn = math.sin(theta)

    rotated_x = x * cs - y * sn
    rotated_y = x * sn + y * cs

    return vector3(rotated_x, rotated_y, 0)
# thanks orlando again


def calculateAngleFromDesiredTorque(moment_arm, force, mmoi, desired_torque):
    if force != 0.0:
        calcval = desired_torque * mmoi / force / moment_arm
        if abs(calcval) > 1.0:
            return 0.0 
        return math.asin(calcval)
    else:
        return 0.0


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

        self.pos += self.vel * dt + 0.5 * accel * (dt * dt)
        self.vel += accel * dt
        
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
        self.vel += K_b * self.vel

        self.vel += y*10
        self.vel /= 11.0

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