from simulation.physics import *
from simulation.dataManagement import *
from simulation.motors import *
from simulation.body import *

# user libraries

from libs.controlMath import *
from libs.ori import *

import numpy as np
import random

import time
from datetime import datetime

datetime_object = datetime.now()

# control

ori: orientation = orientation()

flight_path: flightPath = flightPath()

global setpoint
setpoint: vector3 = vector3()

FSF_pitch: FSF = FSF(6, 4.5)
FSF_yaw: FSF = FSF(6, 4.5)

FSF_position_y: FSF = FSF(7.07106781, 11.93404191)
FSF_position_z: FSF = FSF(7.07106781, 11.93404191)

tvc_command: vector3 = vector3()

global last_accel
last_accel: vector3 = vector3()

global motor_is_lit
motor_is_lit = False

global motor_light_time
motor_light_time = 0.0

KF_x: kalman = kalman()
KF_y: kalman = kalman()
KF_z: kalman = kalman()

global velocity
global position
position: vector3 = vector3()
velocity: vector3 = vector3()

acceleration_inertial: vector3 = vector3()

last_position: vector3 = vector3()

momentum_current: float = 0.0

global apogee 
apogee = False

global impulse_c6
# impulse_c6 = 27.2 # newton-seconds
impulse_c6 = 74.3

def angle_from_desired_accel(thrust, mass, target_accel):
    return np.arcsin(target_accel/(thrust/mass))

# timing

lastIMURead: float = 0.0
imuDT: float = 1 / 500.0

lastGPSRead: float = 0.0
gpsDT: float = 1 / 10.0

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

    DL.addDataPoint("AOA")

    DL.addDataPoint("thrust")
    DL.initCSV(True, True)
    
def init_data1():

    DL.addDataPoint("time")

    DL.addDataPoint("ori_x")
    DL.addDataPoint("ori_y")
    DL.addDataPoint("ori_z")

    DL.addDataPoint("accel_x")
    DL.addDataPoint("accel_y")
    DL.addDataPoint("accel_z")

    DL.addDataPoint("vel_x")
    DL.addDataPoint("vel_y")
    DL.addDataPoint("vel_z")

    DL.addDataPoint("pos_x")
    DL.addDataPoint("pos_y")
    DL.addDataPoint("pos_z")

    DL.addDataPoint("thrust")

    DL.initCSV(True, True)

def record_data1(rocket: rocketBody) -> None:
    datetime_object = datetime.now()
    
    dStr = str(datetime_object)
    
    dStr.replace('-', '/')
    
    # DL.recordVariable("time", f"{dStr}")
    DL.recordVariable("time", rocket.time)

    DL.recordVariable("ori_x", rocket.body.rotation_euler.x * RAD_TO_DEG)
    DL.recordVariable("ori_y", rocket.body.rotation_euler.y * RAD_TO_DEG)
    DL.recordVariable("ori_z", rocket.body.rotation_euler.z * RAD_TO_DEG)

    DL.recordVariable("accel_x", rocket.IMU.accel.x)
    DL.recordVariable("accel_y", rocket.IMU.accel.y)
    DL.recordVariable("accel_z", rocket.IMU.accel.z)

    DL.recordVariable("vel_x", rocket.body.velocity.x)
    DL.recordVariable("vel_y", rocket.body.velocity.y)
    DL.recordVariable("vel_z", rocket.body.velocity.z)

    DL.recordVariable("pos_x", rocket.body.position.x)
    DL.recordVariable("pos_y", rocket.body.position.y)
    DL.recordVariable("pos_z", rocket.body.position.z)

    DL.recordVariable("thrust", rocket.rocket_motor.current_thrust)

    DL.saveData(True)

def record_data(rocket: rocketBody) -> None:

    global setpoint

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

    DL.recordVariable("vel_x_sensed", velocity.x)
    DL.recordVariable("vel_y_sensed", velocity.y)
    DL.recordVariable("vel_z_sensed", velocity.z)

    DL.recordVariable("pos_x", rocket.body.position.x)
    DL.recordVariable("pos_y", rocket.body.position.y)
    DL.recordVariable("pos_z", rocket.body.position.z)

    DL.recordVariable("pos_x_sensed", position.x)
    DL.recordVariable("pos_y_sensed", position.y)
    DL.recordVariable("pos_z_sensed", position.z)

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

    if ( rocket.body.position.x > 0.1):
        DL.recordVariable("AOA", rocket.body.aoa * RAD_TO_DEG)
    else:
        DL.recordVariable("AOA", 0.0)

    DL.recordVariable("thrust", rocket.rocket_motor.current_thrust)

    DL.saveData(True)

def read_imu(rocket: rocketBody) -> None:
    global velocity
    global position
    global motor_is_lit
    global motor_light_time
    global last_accel
    rocket.IMU.readAccel(rocket.body.acceleration_local, rocket.time)
    rocket.IMU.readGyro(rocket.body.rotational_velocity_local, rocket.time)
    
    ori.update(rocket.IMU.oriRates, rocket.time_step*2)

    acceleration_inertial = ori.rotation_quaternion.rotate(rocket.IMU.accel)
    acceleration_inertial.x -= 9.8
    
    acceleration_inertial *= rocket.time_step*2

    KF_x.sFk(rocket.time_step*2)
    KF_y.sFk(rocket.time_step*2)
    KF_z.sFk(rocket.time_step*2)

    KF_x.propogate(acceleration_inertial.x, rocket.time_step*2)
    KF_y.propogate(acceleration_inertial.y, rocket.time_step*2)
    KF_z.propogate(acceleration_inertial.z, rocket.time_step*2)
    
    # position.x = KF_x.get_position()
    # position.y = KF_y.get_position()
    # position.z = KF_z.get_position()
    
    # velocity.x = KF_x.get_velocity()
    # velocity.y = KF_y.get_velocity()
    # velocity.z = KF_z.get_velocity()
    
    position = rocket.body.position
    velocity = rocket.body.velocity
    
    if rocket.IMU.accel.x < last_accel.x - 0.01 and motor_is_lit == False:
        motor_is_lit = True
        motor_light_time = rocket.time
    
    last_accel = rocket.IMU.accel      
    
def read_barometer(rocket: rocketBody) -> None:
    global position
    rocket.barometer.read(rocket.body.position.x, rocket.time)

def read_gps(rocket: rocketBody) -> None:

    rocket.gps.update(rocket.body.position, rocket.body.velocity,
                      rocket.time_step, rocket.time)
    
    Zx = np.matrix([[rocket.gps.measuredPosition.x],
                   [rocket.gps.measuredVelocity.x],
                   [0.0]])
    
    KF_x.sFk(gpsDT)
    
    KF_x.get_kalman_gain()
    KF_x.update_measurement(Zx)
    KF_x.propogate(0.0, gpsDT)
    
    Zy = np.matrix([[rocket.gps.measuredPosition.y],
                   [rocket.gps.measuredVelocity.y],
                   [0.0]])
    
    KF_y.sFk(gpsDT)
    
    KF_y.get_kalman_gain()
    KF_y.update_measurement(Zy)
    KF_y.propogate(0.0, gpsDT)
    
    Zz = np.matrix([[rocket.gps.measuredPosition.z],
                   [rocket.gps.measuredVelocity.z],
                   [0.0]])
    
    KF_z.sFk(gpsDT)
    
    KF_z.get_kalman_gain()
    KF_z.update_measurement(Zz)
    KF_z.propogate(0.0, gpsDT)
vel_at_burn_start_real = vector3()

def TVC_update(rocket: rocketBody) -> None:
    global apogee
    global impulse_c6
    setpoint: vector3 = vector3(0.0, 0.0, 0.0)
    
    ## ignition logic
    
    if velocity.x < 0.0 and position.x > 1.0 and apogee == False:
        apogee = True
        # rocket.body.drag_area = 0.01
        rocket.cp_locaoation = vector3(-0.8, 0.0, 0.0)
        # rocket.dry_mass -= 0.2
        
    velocity_at_burn = vector3(abs(velocity.x) + (5.68*0.78), velocity.y, velocity.z).norm()
    momentum_at_burn = velocity_at_burn * rocket.body.mass
    
    # print(rocket.body.mass)
    
    work_g = (velocity_at_burn*7.02 - ( 0.5 * ((10.9/rocket.body.mass)-5.68)) * (7.02**2))

    # if position.x - abs(work_g) <= 0.5 and apogee:
    #     rocket.rocket_motor.light_motor("descent", rocket.time)
    effective_i = impulse_c6-work_g
    
    # if effective_i < momentum_at_burn and apogee and rocket.rocket_motor.current_thrust < 0.01:
        # vel_at_burn_start_real = rocket.body.velocity
        # rocket.rocket_motor.light_motor("descent", rocket.time)
        # print(rocket.time)



    # if rocket.time > 1:
    #     setpoint = flight_path.getCurrentSetpoint(rocket.time) * DEG_TO_RAD
    # if rocket.time > 4:
    if rocket.time > 6:
        FSF_position_y.setpoint = 1.0
    if rocket.time > 12:
        FSF_position_y.setpoint = 2.5
    FSF_position_y.compute(rocket.body.position.y, rocket.body.velocity.y)
    FSF_position_z.compute(rocket.body.position.z, rocket.body.velocity.z)
    if rocket.rocket_motor.current_thrust > 0.1:
        setpoint.y = -angle_from_desired_accel(rocket.rocket_motor.current_thrust, rocket.body.mass, clamp(FSF_position_z.getOutput(), -5, 5))
        setpoint.z = angle_from_desired_accel(rocket.rocket_motor.current_thrust, rocket.body.mass, clamp(FSF_position_y.getOutput(), -5, 5))
    #     if rocket.body.position.x < 6.0:
    #         setpoint.y = 0.0
    #         setpoint.z = 0.0
    FSF_pitch.setpoint = setpoint.y
    FSF_yaw.setpoint = setpoint.z

    FSF_pitch.compute(rocket.body.rotation_euler.y, rocket.IMU.oriRates.y)
    FSF_yaw.compute(rocket.body.rotation_euler.z, rocket.IMU.oriRates.z)

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
    # put your setup code here, to run once:

    data_list = []

    for point in DL.variableDescriptions:
        data_list.append(point)

    plotter = dataVisualiser()
    plotter.allDataDescriptions = data_list
    DL.fileName = "../data_out.csv"

    init_data()
    
    flight_path.loadFlightPath('../flight_path.csv')

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
        # read_barometer(rocket)
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
    if rocket.time > 1.0:
        cd.motor_fire = "ascent"
    return cd
