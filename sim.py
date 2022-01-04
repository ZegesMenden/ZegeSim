from dataclasses import dataclass

import os

from simulation.physics import *
from simulation.dataManagement import *
from simulation.motors import *
from simulation.sensors import *
from simulation.body import *
from simulation.plotter import *

from flightCode import setup, loop

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

rocket.body.dryMass = settingsLoader.mass
rocket.body.moment_of_inertia = settingsLoader.mmoi
rocket.body.gravity = vector3(-9.8, 0.0, 0.0)

rocket.body.drag_area = settingsLoader.drag_area
rocket.body.drag_coefficient = settingsLoader.drag_coeff
rocket.body.wind_speed = settingsLoader.wind_speed

rocket.IMU.sampleRateAccel = 1 / settingsLoader.imu_accel_read_speed
rocket.IMU.sampleRateGyro = 1 / settingsLoader.imu_gyro_read_speed

rocket.barometer.readDelay = 1 / settingsLoader.baro_read_speed
rocket.rocket_motor.maxIgnitionDelay = settingsLoader.max_ignition_delay

rocket.tvc.linkageRatio = settingsLoader.linkage_ratio
rocket.tvc.max = settingsLoader.max_tvc
rocket.tvc.min = settingsLoader.max_tvc * -1
rocket.tvc_location = settingsLoader.tvc_location
rocket.body.cp_location = settingsLoader.cp_location

rocket.IMU.gyroNoise = vector3(0.5, 0.5, 0.5) * DEG_TO_RAD
rocket.IMU.accelNoise = vector3(0.1, 0.1, 0.1)

rocket.IMU.accelScale = 9.8 * 8
rocket.IMU.gyroScale = 2000 * DEG_TO_RAD

for motor in settingsLoader.motors:
    # print(motor[0], motor[1])
    rocket.rocket_motor.add_motor(motor[1], motor[0])

rocket.body.mass = rocket.body.dryMass + rocket.rocket_motor.totalMotorMass

# simulation

# run startup code
setup(rocket)

while True:

    rocket.time += dt

    rocket.update()

    dataIn = loop(rocket)

    rocket.body.clear()

    rocket.rocket_motor.ignite(dataIn.motor_fire, rocket.time)
    rocket.tvc_position = dataIn.tvc_position    

    if rocket.time > simulation_time:
        break

os.chdir(os.path.dirname(os.path.abspath(__file__)))
p : plotter = plotter()

p.read_header("data_out.csv")
p.create_2d_graph(['time', 'ori_x', 'ori_y', 'ori_z', 'ori_x_sensed', 'ori_y_sensed', 'ori_z_sensed'], "time", "various readings (deg)", True)
p.show_all_graphs()