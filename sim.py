# change flightCode to a different file to change what runs on the rocket!
from flightCode import setup, loop

from dataclasses import dataclass

import os

from simulation.physics import *
from simulation.dataManagement import *
from simulation.motors import *
from simulation.sensors import *
from simulation.body import *
from simulation.plotter import *

dataIn: control_data = control_data()

rocket: rocketBody = rocketBody()

# load settings

settingsLoader = settingsParser()
os.chdir(os.path.dirname(os.path.abspath(__file__)))
settingsLoader.load_settings("settings.yaml")

simulation_time: float = settingsLoader.simulation_time
simulation_time_step: float = settingsLoader.time_step
dt : float = 1 / simulation_time_step
rocket.time_step = dt
rocket.rocket_motor.timeStep = settingsLoader.time_step

rocket.dry_mass = settingsLoader.mass
rocket.body.moment_of_inertia = settingsLoader.mmoi
rocket.body.gravity = vector3(-9.8, 0.0, 0.0)

rocket.body.drag_area = settingsLoader.drag_area
rocket.body.drag_coefficient = settingsLoader.drag_coeff
rocket.body.wind_speed = settingsLoader.wind_speed

rocket.IMU.sampleRateAccel = 1 / settingsLoader.imu_accel_read_speed
rocket.IMU.sampleRateGyro = 1 / settingsLoader.imu_gyro_read_speed

rocket.barometer.readDelay = 1 / settingsLoader.baro_read_speed
rocket.rocket_motor.maxIgnitionDelay = settingsLoader.max_ignition_delay

rocket.tvc.linkage_ratio = settingsLoader.linkage_ratio
rocket.tvc.max = settingsLoader.max_tvc
rocket.tvc.min = settingsLoader.max_tvc * -1
rocket.tvc_location = settingsLoader.tvc_location
rocket.tvc.servo_speed = settingsLoader.tvc_servo_speed
rocket.body.cp_location = settingsLoader.cp_location

rocket.IMU.gyroNoise = vector3(0.5, 0.5, 0.5) * DEG_TO_RAD
rocket.IMU.accelNoise = vector3(0.1, 0.1, 0.1)

rocket.IMU.accelScale = 9.8 * 8
rocket.IMU.gyroScale = 2000 * DEG_TO_RAD

apogee = False

for motor in settingsLoader.motors:
    # print(motor[0], motor[1])
    rocket.rocket_motor.add_motor(motor[1], motor[0])

rocket.body.mass = rocket.dry_mass + rocket.rocket_motor.totalMotorMass

# simulation

# run startup code
setup(rocket)
import time
t_stop = 0.0
t_start = time.time()
while True:

    rocket.time += dt
    rocket.body.clear()

    dataIn = loop(rocket)
    rocket.update()
    rocket.body.clear()

    rocket.rocket_motor.ignite(dataIn.motor_fire, rocket.time)
    rocket.tvc_position = dataIn.tvc_position    

    if rocket.body.position.x > 0.1 and rocket.body.velocity.x < 0.0:
        apogee = True

    if apogee and rocket.body.position.x <= 0.01 and t_stop == 0.0:
        t_stop = rocket.time + 5.0
        
    if t_stop != 0.0 and rocket.time > t_stop:
        break

    if rocket.time > simulation_time:
        break

    if rocket.time + t_start > time.time():
        time.sleep(0.000000001)

os.chdir(os.path.dirname(os.path.abspath(__file__)))
p : plotter = plotter()

p.read_header("data_out.csv")

p.create_2d_graph(['time', 'ori_x', 'ori_y', 'ori_z', 'ori_x_sensed', 'ori_y_sensed', 'ori_z_sensed', 'actuator_out_y', 'actuator_out_z', 'setpoint_x', 'setpoint_y', 'setpoint_z'], "time", "various readings (deg)", True)

p.create_2d_graph(['time', 'vel_x', 'vel_y', 'vel_z', 'vel_x_sensed', 'vel_y_sensed', 'vel_z_sensed'], "time", "velocity (m/s)", True)

p.create_2d_graph(['time', 'pos_x', 'pos_y', 'pos_z', 'pos_x_sensed', 'pos_y_sensed', 'pos_z_sensed'], "time", "position (m)", True)

p.create_2d_graph(['time', 'accel_x', 'accel_y', 'accel_z', 'accel_x_sensed', 'accel_y_sensed', 'accel_z_sensed'], "time", "acceleration (m/s^2)", True)

p.create_2d_graph(['time', 'accel_x_i', 'accel_y_i', 'accel_z_i', 'accel_x_i_sensed', 'accel_y_i_sensed', 'accel_z_i_sensed'], "time", "acceleration (m/s^2)", True)

p.create_3d_graph(['pos_x', 'pos_y', 'pos_z'], 15.0)

p.create_3d_animation(['time', 'pos_x', 'pos_y', 'pos_z'], 15.0, rocket.time)

p.show_all_graphs()