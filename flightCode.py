from simulation.physics import *
from simulation.dataManagement import *
from simulation.motors import *
from simulation.body import *

# user libraries

from libs.controlMath import *
from libs.ori import *

import numpy as np
import random

# control

ori: orientation = orientation()

flight_path: flightPath = flightPath()

global setpoint
setpoint: vector3 = vector3()

FSF_pitch:  FSF = FSF(5, 1)
FSF_yaw:    FSF = FSF(5, 1)

tvc_command: vector3 = vector3()

global velocity
global position
position: vector3 = vector3()
velocity: vector3 = vector3()

acceleration_inertial: vector3 = vector3()

global apogee 
apogee = False

def angle_from_desired_accel(thrust, mass, target_accel):
    return np.arcsin(target_accel/(thrust/mass))

# timing

lastIMURead: float = 0.0
imuDT: float = 1 / 500.0

lastBaroRead: float = 0.0
baroDT: float = 1 / 40.0

lastTVC: float = 0.0
tvcDT: float = 1 / 100.0

lastDatalog: float = 0.0
dataLogDelay: float = 1 / 40.0

DL: dataLogger = dataLogger()

def init_data():

    DL.addDataPoint("time")

    DL.addDataPoint("ori_x")
    DL.addDataPoint("ori_y")
    DL.addDataPoint("ori_z")

    DL.addDataPoint("ori_rate_x")
    DL.addDataPoint("ori_rate_y")
    DL.addDataPoint("ori_rate_z")

    DL.addDataPoint("ori_x_rate_sensed")
    DL.addDataPoint("ori_y_rate_sensed")
    DL.addDataPoint("ori_z_rate_sensed")

    DL.addDataPoint("ori_x_sensed")
    DL.addDataPoint("ori_y_sensed")
    DL.addDataPoint("ori_z_sensed")

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

    DL.addDataPoint("pos_x")
    DL.addDataPoint("pos_y")
    DL.addDataPoint("pos_z")

    DL.addDataPoint("baro_alt")
    DL.addDataPoint("baro_vel")

    DL.addDataPoint("actuator_out_y")
    DL.addDataPoint("actuator_out_z")

    DL.addDataPoint("setpoint_x")
    DL.addDataPoint("setpoint_y")
    DL.addDataPoint("setpoint_z")

    DL.addDataPoint("AOA")

    DL.addDataPoint("thrust")

    DL.fileName = "../data_out.csv"
    DL.initCSV(True, True)

def record_data(rocket: rocketBody) -> None:

    global setpoint
    global lastDatalog

    DL.recordVariable("time", rocket.time)

    DL.recordVariable("ori_x", rocket.body.rotation_euler.x * RAD_TO_DEG)
    DL.recordVariable("ori_y", rocket.body.rotation_euler.y * RAD_TO_DEG)
    DL.recordVariable("ori_z", rocket.body.rotation_euler.z * RAD_TO_DEG)

    DL.recordVariable("ori_rate_x", rocket.body.rotational_velocity.x * RAD_TO_DEG)
    DL.recordVariable("ori_rate_y", rocket.body.rotational_velocity.y * RAD_TO_DEG)
    DL.recordVariable("ori_rate_z", rocket.body.rotational_velocity.z * RAD_TO_DEG)

    DL.recordVariable("ori_x_sensed", ori.rotaiton_euler.x * RAD_TO_DEG)
    DL.recordVariable("ori_y_sensed", ori.rotaiton_euler.y * RAD_TO_DEG)
    DL.recordVariable("ori_z_sensed", ori.rotaiton_euler.z * RAD_TO_DEG)

    DL.recordVariable("ori_x_rate_sensed", rocket.IMU.oriRates.x * RAD_TO_DEG)
    DL.recordVariable("ori_y_rate_sensed", rocket.IMU.oriRates.y * RAD_TO_DEG)
    DL.recordVariable("ori_z_rate_sensed", rocket.IMU.oriRates.z * RAD_TO_DEG)

    DL.recordVariable("accel_x", rocket.body.acceleration_local.x)
    DL.recordVariable("accel_y", rocket.body.acceleration_local.y)
    DL.recordVariable("accel_z", rocket.body.acceleration_local.z)

    DL.recordVariable("accel_x_sensed", rocket.IMU.accel.x)
    DL.recordVariable("accel_y_sensed", rocket.IMU.accel.y)
    DL.recordVariable("accel_z_sensed", rocket.IMU.accel.z)

    DL.recordVariable("accel_x_i", rocket.body.acceleration.x)
    DL.recordVariable("accel_y_i", rocket.body.acceleration.y)
    DL.recordVariable("accel_z_i", rocket.body.acceleration.z)

    DL.recordVariable("accel_x_i_sensed", acceleration_inertial.x)
    DL.recordVariable("accel_y_i_sensed", acceleration_inertial.y)
    DL.recordVariable("accel_z_i_sensed", acceleration_inertial.z)

    DL.recordVariable("vel_x", rocket.body.velocity.x)
    DL.recordVariable("vel_y", rocket.body.velocity.y)
    DL.recordVariable("vel_z", rocket.body.velocity.z)

    DL.recordVariable("pos_x", rocket.body.position.x)
    DL.recordVariable("pos_y", rocket.body.position.y)
    DL.recordVariable("pos_z", rocket.body.position.z)

    DL.recordVariable("baro_alt", rocket.barometer.altitude)
    DL.recordVariable("baro_vel", rocket.barometer.velocity)

    DL.recordVariable("actuator_out_y", rocket.tvc.position.y * RAD_TO_DEG)
    DL.recordVariable("actuator_out_z", rocket.tvc.position.z * RAD_TO_DEG)

    DL.recordVariable("setpoint_x", setpoint.x * RAD_TO_DEG)
    DL.recordVariable("setpoint_y", setpoint.y * RAD_TO_DEG)
    DL.recordVariable("setpoint_z", setpoint.z * RAD_TO_DEG)

    if ( rocket.body.position.x > 0.1):
        DL.recordVariable("AOA", rocket.body.aoa * RAD_TO_DEG)
    else:
        DL.recordVariable("AOA", 0.0)

    DL.recordVariable("thrust", rocket.rocket_motor.current_thrust)

    DL.saveData(True)

    lastDatalog = rocket.time

def read_imu(rocket: rocketBody) -> None:
    rocket.IMU.readAccel(rocket.body.acceleration_local, rocket.time)
    rocket.IMU.readGyro(rocket.body.rotational_velocity_local, rocket.time)
    
    ori.update(rocket.IMU.oriRates, rocket.time_step*2)

    acceleration_inertial = ori.rotation_quaternion.rotate(rocket.IMU.accel)
    acceleration_inertial.x -= 9.8

    
def read_barometer(rocket: rocketBody) -> None:
    global position
    rocket.barometer.read(rocket.body.position.x, rocket.time)

def TVC_update(rocket: rocketBody) -> None:

    global setpoint

    # setpoint = flight_path.getCurrentSetpoint(rocket.time)

    FSF_pitch.setpoint = 1 * DEG_TO_RAD
    # FSF_yaw.setpoint = setpoint.z * DEG_TO_RAD

    FSF_pitch.compute(rocket.body.rotation_euler.y, rocket.body.rotational_velocity.y)
    FSF_yaw.compute(rocket.body.rotation_euler.z, rocket.body.rotational_velocity.z)

    TVC_y = calculateAngleFromDesiredTorque(rocket.tvc_location.x, rocket.rocket_motor.current_thrust, rocket.body.moment_of_inertia.y, FSF_pitch.getOutput())
    TVC_z = calculateAngleFromDesiredTorque(rocket.tvc_location.x, rocket.rocket_motor.current_thrust, rocket.body.moment_of_inertia.z, FSF_yaw.getOutput())

    cr = math.cos(-rocket.body.rotation_euler.x)
    sr = math.sin(-rocket.body.rotation_euler.x)

    tvcy = TVC_y * cr - TVC_z * sr
    tvcz = TVC_y * sr + TVC_z * cr

    return vector3(0.0, tvcy, tvcz)
    
    # return vector3(0.0, TVC_y, TVC_z)
    return vector3(0.0, 0.0, 0.0)

def setup(rocket: rocketBody) -> None:

    flight_path.loadFlightPath("flight_path.csv")

    init_data()


    pass

def loop(rocket: rocketBody) -> control_data:        

    if rocket.time > lastIMURead + imuDT:
        read_imu(rocket)
    
    if rocket.time > lastBaroRead + baroDT:
        read_barometer(rocket)

    if rocket.time > lastDatalog + dataLogDelay:
        record_data(rocket)

    cd: control_data = control_data()

    # if rocket.time > lastTVC + tvcDT:
    cd.tvc_position = TVC_update(rocket)

    cd.motor_fire = "ascent"

    return cd
