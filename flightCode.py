# simulation libraries

from simulation.physics import *
from simulation.dataManagement import *
from simulation.motors import *
from simulation.body import *

# user libraries

from libs.controlMath import *
from libs.ori import *

import math
import random

# control

NAV = nav()

NAV.posKF_x.Q = 0.25
NAV.posKF_x.R = 0.15

NAV.posKF_y.Q = 0.25
NAV.posKF_y.R = 0.15

NAV.posKF_z.Q = 0.25
NAV.posKF_z.R = 0.15


global setpoint
setpoint: vector3 = vector3()

FSF_pitch: FSF = FSF(3.16227766, 1.97902394)
FSF_yaw: FSF = FSF(3.16227766, 1.97902394)

tvc_command: vector3 = vector3()

# timing

lastIMURead: float = 0.0
imuDT: float = 1 / 500.0

lastGPSRead: float = 0.0
gpsDT: float = 1 / 10.0

lastBaroRead: float = 0.0
baroDT: float = 1 / 40.0

lastTVC: float = 0.0
tvcDT: float = 1 / 50.0

lastDatalog: float = 0.0
dataLogDelay: float = 1 / 50.0

DL: dataLogger = dataLogger()


def init_data():

    DL.addDataPoint("time")

    DL.addDataPoint("ori_x")
    DL.addDataPoint("ori_y")
    DL.addDataPoint("ori_z")

    DL.addDataPoint("ori_rate_x")
    DL.addDataPoint("ori_rate_y")
    DL.addDataPoint("ori_rate_z")

    DL.addDataPoint("ori_x_sensed")
    DL.addDataPoint("ori_y_sensed")
    DL.addDataPoint("ori_z_sensed")

    DL.addDataPoint("ori_x_rate_sensed")
    DL.addDataPoint("ori_y_rate_sensed")
    DL.addDataPoint("ori_z_rate_sensed")

    DL.addDataPoint("accel_x")
    DL.addDataPoint("accel_y")
    DL.addDataPoint("accel_z")

    DL.addDataPoint("accel_x_sensed")
    DL.addDataPoint("accel_y_sensed")
    DL.addDataPoint("accel_z_sensed")

    DL.addDataPoint("accel_x_i")
    DL.addDataPoint("accel_y_i")
    DL.addDataPoint("accel_z_i")

    DL.addDataPoint("accel_x_i_sensed")
    DL.addDataPoint("accel_y_i_sensed")
    DL.addDataPoint("accel_z_i_sensed")

    DL.addDataPoint("vel_x")
    DL.addDataPoint("vel_y")
    DL.addDataPoint("vel_z")

    DL.addDataPoint("vel_x_sensed")
    DL.addDataPoint("vel_y_sensed")
    DL.addDataPoint("vel_z_sensed")

    DL.addDataPoint("pos_x")
    DL.addDataPoint("pos_y")
    DL.addDataPoint("pos_z")

    DL.addDataPoint("pos_x_sensed")
    DL.addDataPoint("pos_y_sensed")
    DL.addDataPoint("pos_z_sensed")

    DL.addDataPoint("gps_pos_x")
    DL.addDataPoint("gps_pos_y")
    DL.addDataPoint("gps_pos_z")

    DL.addDataPoint("gps_vel_x")
    DL.addDataPoint("gps_vel_y")
    DL.addDataPoint("gps_vel_z")

    DL.addDataPoint("baro_alt")
    DL.addDataPoint("baro_vel")

    DL.addDataPoint("actuator_out_y")
    DL.addDataPoint("actuator_out_z")

    DL.addDataPoint("setpoint_x")
    DL.addDataPoint("setpoint_y")
    DL.addDataPoint("setpoint_z")

    DL.addDataPoint("thrust")
    DL.initCSV(True, True)

    # DL.addDataPoint("")
    # DL.addDataPoint("")
    # DL.addDataPoint("")


data_list = []

for point in DL.variableDescriptions:
    data_list.append(point)

plotter = dataVisualiser()
plotter.allDataDescriptions = data_list
DL.fileName = "../data_out.csv"


def record_data(rocket: rocketBody) -> None:

    DL.recordVariable("time", rocket.time)

    DL.recordVariable("ori_x", rocket.body.rotation_euler.x * RAD_TO_DEG)
    DL.recordVariable("ori_y", rocket.body.rotation_euler.y * RAD_TO_DEG)
    DL.recordVariable("ori_z", rocket.body.rotation_euler.z * RAD_TO_DEG)

    DL.recordVariable("ori_rate_x", rocket.body.rotational_velocity.x * RAD_TO_DEG)
    DL.recordVariable("ori_rate_y", rocket.body.rotational_velocity.y * RAD_TO_DEG)
    DL.recordVariable("ori_rate_z", rocket.body.rotational_velocity.z * RAD_TO_DEG)

    DL.recordVariable("ori_x_sensed", NAV.orientation_euler.x * RAD_TO_DEG)
    DL.recordVariable("ori_y_sensed", NAV.orientation_euler.y * RAD_TO_DEG)
    DL.recordVariable("ori_z_sensed", NAV.orientation_euler.z * RAD_TO_DEG)

    DL.recordVariable("ori_x_rate_sensed", NAV.oriRates.x * RAD_TO_DEG)
    DL.recordVariable("ori_y_rate_sensed", NAV.oriRates.y * RAD_TO_DEG)
    DL.recordVariable("ori_z_rate_sensed", NAV.oriRates.z * RAD_TO_DEG)

    DL.recordVariable("accel_x", rocket.body.acceleration_local.x)
    DL.recordVariable("accel_y", rocket.body.acceleration_local.y)
    DL.recordVariable("accel_z", rocket.body.acceleration_local.z)

    DL.recordVariable("accel_x_sensed", NAV.accelerationLocal.x)
    DL.recordVariable("accel_y_sensed", NAV.accelerationLocal.y)
    DL.recordVariable("accel_z_sensed", NAV.accelerationLocal.z)

    DL.recordVariable("accel_x_i", rocket.body.acceleration_inertial.x)
    DL.recordVariable("accel_y_i", rocket.body.acceleration_inertial.y)
    DL.recordVariable("accel_z_i", rocket.body.acceleration_inertial.z)

    DL.recordVariable("accel_x_i_sensed", NAV.accelerationInertial.x)
    DL.recordVariable("accel_y_i_sensed", NAV.accelerationInertial.y)
    DL.recordVariable("accel_z_i_sensed", NAV.accelerationInertial.z)

    DL.recordVariable("vel_x", rocket.body.velocity.x)
    DL.recordVariable("vel_y", rocket.body.velocity.y)
    DL.recordVariable("vel_z", rocket.body.velocity.z)

    DL.recordVariable("vel_x_sensed", NAV.velocityInertial.x)
    DL.recordVariable("vel_y_sensed", NAV.velocityInertial.y)
    DL.recordVariable("vel_z_sensed", NAV.velocityInertial.z)

    DL.recordVariable("pos_x", rocket.body.position.x)
    DL.recordVariable("pos_y", rocket.body.position.y)
    DL.recordVariable("pos_z", rocket.body.position.z)

    DL.recordVariable("pos_x_sensed", NAV.positionInertial.x)
    DL.recordVariable("pos_y_sensed", NAV.positionInertial.y)
    DL.recordVariable("pos_z_sensed", NAV.positionInertial.z)

    DL.recordVariable("gps_pos_x", rocket.gps.measuredPosition.x)
    DL.recordVariable("gps_pos_y", rocket.gps.measuredPosition.y)
    DL.recordVariable("gps_pos_z", rocket.gps.measuredPosition.z)

    DL.recordVariable("gps_vel_x", rocket.gps.measuredVelocity.x)
    DL.recordVariable("gps_vel_y", rocket.gps.measuredVelocity.y)
    DL.recordVariable("gps_vel_z", rocket.gps.measuredVelocity.z)

    DL.recordVariable("baro_alt", rocket.barometer.altitude)
    DL.recordVariable("baro_vel", rocket.barometer.velocity)

    DL.recordVariable("actuator_out_y", rocket.tvc.position.y * RAD_TO_DEG)
    DL.recordVariable("actuator_out_z", rocket.tvc.position.z * RAD_TO_DEG)

    DL.recordVariable("setpoint_x", setpoint.x * RAD_TO_DEG)
    DL.recordVariable("setpoint_y", setpoint.y * RAD_TO_DEG)
    DL.recordVariable("setpoint_z", setpoint.z * RAD_TO_DEG)

    DL.recordVariable("thrust", rocket.rocket_motor.currentThrust)

    DL.saveData(True)


def read_imu(rocket: rocketBody) -> None:
    trueOriRates: vector3 = rocket.body.rotation_quaternion.conj().rotateVector(rocket.body.rotational_velocity)

    rocket.IMU.readAccel(rocket.body.acceleration_local, rocket.time)
    rocket.IMU.readGyro(trueOriRates, rocket.time)

    # if NAV.debiased:
    NAV.update(rocket.IMU.accel, rocket.IMU.oriRates,
                rocket.body.gravity, rocket.time_step)
    # else:
    #     if rocket.time < 1.0:
    #         NAV.measureDebias(rocket.IMU.accel, rocket.IMU.oriRates)
    #     else:
    #         NAV.debias()


def read_barometer(rocket: rocketBody) -> None:

    rocket.barometer.read(rocket.body.position.x, rocket.time)
    NAV.passBarometerData(rocket.barometer.altitude,
                          rocket.barometer.velocity, rocket.time)


def read_gps(rocket: rocketBody) -> None:

    rocket.gps.update(rocket.body.position, rocket.body.velocity,
                      rocket.time_step, rocket.time)
    NAV.passGPSData(rocket.gps.measuredPosition)


def TVC_update(rocket: rocketBody) -> None:

    FSF_pitch.setpoint = 5 * DEG_TO_RAD

    FSF_pitch.compute(NAV.orientation_euler.y, NAV.oriRates.y)
    FSF_yaw.compute(NAV.orientation_euler.z, NAV.oriRates.z)

    TVC_y = calculateAngleFromDesiredTorque(rocket.tvc_location.x, NAV.accelerationLocal.x, rocket.body.moment_of_inertia.y, FSF_pitch.getOutput())
    TVC_z = calculateAngleFromDesiredTorque(rocket.tvc_location.x, NAV.accelerationLocal.x, rocket.body.moment_of_inertia.z, FSF_yaw.getOutput())

    cr = math.cos(-NAV.orientation_euler.x)
    sr = math.sin(-NAV.orientation_euler.x)

    tvcy = TVC_y * cr - TVC_z * sr
    tvcz = TVC_y * sr + TVC_z * cr

    return vector3(0.0, tvcy, tvcz)
    # return vector3(0.0, TVC_y, TVC_z)


def setup(rocket: rocketBody) -> None:
    # put your setup code here, to run once:

    init_data()

    pass


def loop(rocket: rocketBody) -> control_data:
    global lastIMURead
    global lastBaroRead
    global lastGPSRead
    global lastTVC
    global tvc_command
    global lastDatalog

    if rocket.time > lastIMURead + imuDT:
        read_imu(rocket)

        lastIMURead = rocket.time

    if rocket.time > lastBaroRead + baroDT:
        read_barometer(rocket)

        lastBaroRead = rocket.time

    if rocket.time > lastGPSRead + gpsDT:

        read_gps(rocket)
        lastGPSRead = rocket.time

    if rocket.time > lastTVC + tvcDT:

        tvc_command = TVC_update(rocket)
        # tvc_command = vector3()

        lastTVC = rocket.time

    if rocket.time > lastDatalog + dataLogDelay:

        record_data(rocket)

        lastDatalog = rocket.time

    cd: control_data = control_data()
    # if tvc_command.y != 0.0:
    #     print(tvc_command)
    cd.tvc_position = tvc_command
    cd.motor_fire = "ascent"
    return cd
