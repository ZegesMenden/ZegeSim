import random

d12_thrust = [
    [0.049, 2.569],
    [0.116, 9.369],
    [0.184, 17.275],
    [0.237, 24.258],
    [0.282, 29.73],
    [0.297, 27.01],
    [0.311, 22.589],
    [0.322, 17.99],
    [0.348, 14.126],
    [0.386, 12.099],
    [0.442, 10.808],
    [0.546, 9.876],
    [0.718, 9.306],
    [0.879, 9.105],
    [1.066, 8.901],
    [1.257, 8.698],
    [1.436, 8.31],
    [1.59, 8.294],
    [1.612, 4.613],
    [1.65, 0]
]

e6_thrust = [
    [0.056, 18.59],
    [0.112, 20.12],
    [0.168, 17.575],
    [0.307, 14.38],
    [0.531, 10.45],
    [0.894, 7.696],
    [1.146, 6.244],
    [1.691, 5.808],
    [2.836, 5.663],
    [3.898, 5.517],
    [4.275, 5.227],
    [4.415, 4.937],
    [5.058, 5.082],
    [5.519, 5.227],
    [5.603, 6.679],
    [5.729, 3.921],
    [5.882, 2.323],
    [5.966, 1.016],
    [6.06, 0]
]

e6_rct_thrust = [
    [0, 0],
    [0.047, 10.866],
    [0.127, 11.693],
    [0.19, 11.9],
    [0.316, 11.622],
    [0.522, 10.593],
    [0.743, 9.287],
    [0.996, 7.842],
    [1.249, 6.19],
    [1.47, 5.296],
    [1.787, 4.747],
    [2.372, 4.471],
    [3.02, 4.403],
    [3.747, 4.264],
    [4.49, 4.403],
    [5.375, 4.333],
    [6.087, 4.264],
    [6.719, 4.264],
    [6.877, 4.196],
    [6.957, 3.783],
    [7.004, 2.614],
    [7.036, 1.513],
    [7.083, 0.55],
    [7.12, 0]
]

e12_thrust = [
    [0.052, 5.045],
    [0.096, 9.910],
    [0.196, 24.144],
    [0.251, 31.351],
    [0.287, 32.973],
    [0.300, 29.910],
    [0.344, 17.117],
    [0.370, 14.414],
    [0.400, 12.973],
    [0.500, 11.712],
    [0.600, 11.171],
    [0.700, 10.631],
    [0.800, 10.09],
    [0.900, 9.73],
    [1.000, 9.55],
    [1.101, 9.91],
    [1.200, 9.55],
    [1.300, 9.73],
    [1.400, 9.73],
    [1.500, 9.73],
    [1.600, 9.73],
    [1.700, 9.55],
    [1.800, 9.73],
    [1.900, 9.73],
    [2.000, 9.55],
    [2.100, 9.55],
    [2.200, 9.73],
    [2.300, 9.19],
    [2.375, 9.37],
    [2.400, 5.95],
    [2.440, 0.0]
]

f15_thrust = [
    [0.063, 2.127],
    [0.118, 4.407],
    [0.158, 8.359],
    [0.228, 13.68],
    [0.340, 20.82],
    [0.386, 26.75],
    [0.425, 25.38],
    [0.481, 22.19],
    [0.583, 17.93],
    [0.883, 16.11],
    [1.191, 14.59],
    [1.364, 15.35],
    [1.569, 15.65],
    [1.727, 14.74],
    [2.00, 14.28],
    [2.39, 13.68],
    [2.68, 13.08],
    [2.96, 13.07],
    [3.25, 13.05],
    [3.35, 13.0],
    [3.39, 7.30],
    [3.40, 0.00]
]

f10_thrust = [
    [0.015, 28.22],
    [0.077, 26.082],
    [0.201, 24.934],
    [0.31, 22.806],
    [0.464, 20.183],
    [0.573, 17.886],
    [0.789, 16.075],
    [1.068, 13.946],
    [1.393, 12.63],
    [1.718, 11.155],
    [2.166, 9.844],
    [2.677, 9.515],
    [3.311, 9.187],
    [3.683, 8.859],
    [3.791, 9.679],
    [4.101, 9.679],
    [4.658, 9.515],
    [5.168, 9.023],
    [5.725, 9.023],
    [6.112, 8.531],
    [6.329, 8.859],
    [6.499, 7.546],
    [6.685, 5.742],
    [6.778, 4.921],
    [6.917, 2.625],
    [7.025, 1.312],
    [7.13, 0]
]

g12_RCT_thrust = [
    [0.03, 18.549],
    [0.117, 19.96],
    [0.239, 20.64],
    [0.362, 20.111],
    [0.519, 18.982],
    [0.694, 17.138],
    [0.886, 15.02],
    [1.131, 13.186],
    [1.375, 11.915],
    [1.689, 11.069],
    [2.021, 10.363],
    [2.422, 10.232],
    [3.172, 9.677],
    [4.114, 9.267],
    [5.039, 8.857],
    [6.137, 8.733],
    [7.132, 8.607],
    [7.795, 8.335],
    [7.952, 8.196],
    [8.074, 8.055],
    [8.179, 6.924],
    [8.319, 4.661],
    [8.476, 1.973],
    [8.55, 0]
]

g12st_thrust = [
    [0.042, 33.827],
    [0.104, 30.173],
    [0.23, 28.009],
    [0.543, 22.326],
    [0.836, 16.102],
    [1.024, 12.448],
    [1.379, 10.96],
    [2.006, 10.148],
    [4.054, 9.742],
    [6.269, 10.148],
    [6.415, 10.148],
    [6.582, 10.148],
    [11.973, 9.742],
    [12.475, 9.742],
    [12.663, 9.607],
    [12.83, 9.066],
    [12.913, 7.713],
    [12.934, 5.412],
    [13.018, 2.03],
    [13.06, 0.0]
]

g11_thrust = [
    [0.084, 30.444],
    [0.105, 28.414],
    [0.209, 27.738],
    [0.419, 24.085],
    [0.753, 18.402],
    [0.9, 14.748],
    [1.046, 11.907],
    [1.444, 10.013],
    [2.051, 9.742],
    [3.034, 9.201],
    [4.018, 9.471],
    [5.483, 9.201],
    [5.713, 8.93],
    [9.375, 9.742],
    [9.501, 8.93],
    [11.049, 9.742],
    [12.263, 9.336],
    [13.288, 8.66],
    [13.456, 7.307],
    [13.602, 4.6],
    [13.77, 1.624],
    [13.937, 0.0]
]

g8st_thrust = [
    [0.038, 5.121],
    [0.039, 8.069],
    [0.188, 9.828],
    [0.414, 10.397],
    [0.715, 10.19],
    [1.354, 9.517],
    [2.069, 9.155],
    [3.424, 8.793],
    [4.552, 8.431],
    [6.057, 8.276],
    [6.81, 8.069],
    [7.713, 8.121],
    [9.03, 8.017],
    [9.97, 7.966],
    [10.76, 7.914],
    [14.222, 7.397],
    [14.335, 7.19],
    [15.764, 7.138],
    [16.404, 6.983],
    [16.554, 7.5],
    [16.63, 6.724],
    [16.818, 5.69],
    [16.968, 3.414],
    [17.119, 1.655],
    [17.269, 0.0]
]

h13st_thrust = [
    [0.005, 0.107],
    [0.024, 2.636],
    [0.035, 18.978],
    [0.081, 32.724],
    [0.147, 36.421],
    [0.379, 44.529],
    [0.452, 23.851],
    [0.566, 18.890],
    [0.818, 16.728],
    [1.286, 15.676],
    [2.114, 14.753],
    [3.230, 14.032],
    [4.382, 13.926],
    [5.786, 13.469],
    [7.082, 13.119],
    [8.666, 12.916],
    [10.286, 12.820],
    [12.086, 12.612],
    [13.598, 12.333],
    [14.750, 11.908],
    [15.230, 11.078],
    [15.302, 6.048],
    [15.432, 0]
]

# making a better thrust curve by interpolating the thrust curve data


def interpolateThrust(thrust_curve, timeStep):
    thrustList = []
    lPoint = [0, 0]
    for point in thrust_curve:
        if point[0] > 0:
            thrustDiff = point[1] - lPoint[1]
            timeDiff = point[0] - lPoint[0]
            stepsNeeded = timeDiff * timeStep

            if stepsNeeded > 0:
                adder = thrustDiff / stepsNeeded

                i = 0

                thrustToAdd = lPoint[1]

                while i < stepsNeeded:
                    i += 1
                    thrustToAdd += adder
                    thrustList.append(thrustToAdd)

        lPoint = point
    return thrustList


stringMotorType = {
    "d12": d12_thrust,
    "e6": e6_thrust,
    "e6_rct": e6_rct_thrust,
    "e12": e12_thrust,
    "f15": f15_thrust,
    "f10": f10_thrust,
    "g12_rct": g12_RCT_thrust,
    "g12_st": g12st_thrust,
    "g11": g11_thrust,
    "g8_st": g8st_thrust,
    "h13": h13st_thrust
}


class motorType(enumerate):
    d12 = d12_thrust
    e6 = e6_thrust
    e6_rct = e6_rct_thrust
    e12 = e12_thrust
    f15 = f15_thrust
    f10 = f10_thrust
    g12_rct = g12_RCT_thrust
    g12_st = g12st_thrust
    g11 = g11_thrust
    g8_st = g8st_thrust
    h13 = h13st_thrust


class rocketMotor:
    def __init__(self, timeStep):
        self.motorNames = []
        self.thrustLists = {}
        self.currentThrust = 0.0
        self.ignitionTimes = {}
        self.ignitionDelays = {}
        self.timeStep = timeStep
        self.maxIgnitionDelay = 0.0
        self.totalMotorMass = 0.0
        self.lastTime = 0.0
        self.isLit = {}
        self.throttlePercent = 1.0  # 1 = full throttle

    def add_motor(self, motor, motorName):
        self.totalMotorMass += 0.025
        self.motorNames.append(str(motorName))
        if isinstance(motor, str):
            self.thrustLists[str(motorName)] = interpolateThrust(
                stringMotorType[motor], self.timeStep)
        else:
            self.thrustLists[str(motorName)] = interpolateThrust(
                motor, self.timeStep)
        self.ignitionDelays[str(motorName)] = random.randint(
            80, 100) / 100 * self.maxIgnitionDelay
        self.ignitionTimes[str(motorName)] = 0.0
        self.isLit[str(motorName)] = False

    def ignite(self, motor, time):
        if str(motor) in self.motorNames:
            if self.isLit[str(motor)] == False:
                if motor == "ascent":
                    self.ignitionTimes[str(motor)] = time * self.timeStep
                else:
                    self.ignitionTimes[str(motor)] = (
                        time + self.ignitionDelays[str(motor)]) * self.timeStep
                self.isLit[str(motor)] = True

    def update(self, time):
        dt = time - self.lastTime
        for motor in self.motorNames:
            if self.isLit[motor] == True:
                counter = int((time * self.timeStep) -
                              self.ignitionTimes[motor])
                if counter > 0 and counter < len(self.thrustLists[motor]):
                    self.currentThrust = self.thrustLists[motor][counter] * \
                        self.throttlePercent
                    self.totalMotorMass -= 0.004 * dt
                else:
                    self.currentThrust = 0.0

        self.lastTime = time

    def throttle(self, percent):
        self.throttlePercent = 1.0 - percent
