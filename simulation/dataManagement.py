import os
import csv
import yaml
from simulation.physics import *


@dataclass
class control_data:

    """Class storing all the control outputs of the rocket

    Params:

    tvc_position - a vector represenging the position of the rocket's thrust vector control mount in radians

    reaction_wheel_output - a torque in newton-meters representing the torque from a reaction wheel

    motor_fire - the name of the motor you want to fire, leave blank to not fire any motor
    """

    tvc_position: vector3 = vector3()
    reaction_wheel_output: float = 0.0

    motor_fire: str = ""


class dataLogger:
    def __init__(self):
        self.variables = 0
        self.variableDescriptions = []
        self.currentLog = {}
        self.loggedData = []
        self.initialized = False
        self.fileName = ""
        pass

    def addDataPoint(self, variableName):
        """Adds a data point to the logger object. Datapoints are added sequentially, so add your variables in the same sequence that you want them to show up in on the CSV"""
        if self.initialized == False:
            if str(variableName) in self.currentLog:
                raise IndexError("datapoiont already initialized")
            else:
                self.variables += 1
                self.variableDescriptions.append(variableName)
                self.currentLog[variableName] = None
        else:
            raise IndexError("file already initialized!")

    def recordVariable(self, variableName, data):
        """records a variable to the current log, DOES NOT LOG AUTOMATICALLY"""
        if str(variableName) in self.currentLog:
            # if self.currentLog[str(variableName)] != None:
            #     raise Warning(f'data point {str(variableName)} is being overwritten!')
            self.currentLog[str(variableName)] = data
        else:
            raise IndexError("datapoint not initialized")

    def initCSV(self, makeFile, overWrite):
        """Initializes the CSV file and prepares it for writing."""
        self.initialized = True

        os.chdir(os.path.dirname(os.path.abspath(__file__)))

        if os.path.exists(str(self.fileName)):

            f = open(str(self.fileName), "r")

            if not f.read():
                f.close()

                f = open(str(self.fileName), "w")
                outString = ""
                for varName in self.variableDescriptions:
                    outString += varName
                    outString += ","

                f.write(outString[0:-1])

                f.write('\n')
            else:
                if overWrite == True:
                    f.close()

                    f = open(str(self.fileName), "w")
                    outString = ""
                    for varName in self.variableDescriptions:
                        outString += varName
                        outString += ","

                    f.write(outString[0:-1])

                    f.write('\n')
                if overWrite == False:
                    raise OSError("csv file is not empty!")

        else:
            if makeFile == True:
                f = open(str(self.fileName), "w")

                f.close()
            else:
                raise OSError("csv file not found!")

    def saveData(self, clearData):
        outString = ""
        for datapoint in self.currentLog:
            currentVar = self.currentLog[str(datapoint)]
            if currentVar == None:
                outString += "0"
            else:
                outString += str(currentVar)
            outString += ","
            if clearData == True:
                self.currentLog[str(datapoint)] = None
        f = open(str(self.fileName), "a")
        f.write(outString[0:-1] + "\n")
        # f.write('\n')

    def getVariable(self, variableName):
        if str(variableName) in self.currentLog:
            return self.currentLog[str(variableName)]
        else:
            raise IndexError("datapoint not initialized")


class dataVisualiser:
    def __init__(self):
        self.allDataDescriptions = []

    def graph_from_csv(self, datapoints):
        descriptionNum = 0
        pointsToLog = []

        for description in self.allDataDescriptions:
            for requestedDatapoint in datapoints:
                if str(description) == str(requestedDatapoint):
                    pointsToLog.append(descriptionNum)
            descriptionNum += 1

        with open('data_out.csv', newline='\n') as pathFile:
            reader = csv.reader(pathFile, delimiter=',', quotechar='"')
            logList = []
            dataOut = []
            for index, row in enumerate(reader):
                for point in pointsToLog:
                    logList.append(row[point])
                if index == 0:
                    for x in logList:
                        dataOut.append([])

                if index > 0:
                    for index, point in enumerate(dataOut):
                        point.append(float(row[pointsToLog[index]]))
                logList = []

        return dataOut


class flightPath:

    def __init__(self):
        self.setpoint = 0.0
        self.setpoints = []
        self.currentSetpoint = vector3(0, 0, 0)

    def loadFlightPath(self, fName):
        with open(fName, newline='\n') as pathFile:
            reader = csv.reader(pathFile, delimiter=',', quotechar='"')
            for row in reader:
                self.setpoints.append([float(row[0]), float(
                    row[1]), float(row[2]), float(row[3])])

    def getCurrentSetpoint(self, time):

        setpointLast = [0, 0, 0, 0]
        setpointFuture = [0, 0, 0, 0]

        for index, datapoint in enumerate(self.setpoints):

            if datapoint[0] < time:
                setpointLast = datapoint

            if datapoint[0] > time and setpointFuture == [0, 0, 0, 0]:
                setpointFuture = datapoint

            if index == len(self.setpoints) - 1 and setpointFuture == [0, 0, 0, 0]:
                setpointFuture = self.setpoint[-1]

        if setpointFuture != [0, 0, 0, 0]:

            timeDiff = setpointFuture[0] - setpointLast[0]
            setpointDiff = vector3(setpointFuture[1], setpointFuture[2], setpointFuture[3]) - vector3(
                setpointLast[1], setpointLast[2], setpointLast[3])

            rateOfChange = vector3(0.0, 0.0, 0.0)
            rateOfChange = setpointDiff / timeDiff
            self.currentSetpoint = vector3(
                setpointLast[1], setpointLast[2], setpointLast[3]) + rateOfChange * (time - setpointLast[0])

        return self.currentSetpoint


class settingsParser:

    def __init__(self):

        self.motors: list = []
        self.max_ignition_delay: float = 0.0

        self.time_step: float = 0.0
        self.simulation_time: float = 0.0

        self.imu_gyro_read_speed: float = 0.0
        self.imu_accel_read_speed: float = 0.0
        self.gps_read_speed: float = 0.0
        self.baro_read_speed: float = 0.0

        self.mass: float = 0.0
        self.mmoi: vector3 = vector3()

        self.drag_area: float = 0.0
        self.drag_coeff: float = 0.0

        self.wind_speed: vector3 = vector3()

        self.tvc_noise: float = 0.0

        self.tvc_servo_speed: float = 0.0
        self.linkage_ratio: float = 0.0

        self.max_tvc: vector3 = vector3()

        pass

    def load_settings(self, file_name: str):

        with open(file_name, "r") as settingsFile:

            settings = yaml.load(settingsFile, Loader=yaml.FullLoader)

        config = settings["settings"]

        for point in config["motors"]:
            self.motors.append([point, config["motors"][point]])

        self.time_step = float(config["timeStep"])
        self.simulation_time = float(config["simTime"])

        self.imu_gyro_read_speed = float(config["gyroSpeed"])
        self.imu_accel_read_speed = float(config["accelSpeed"])
        self.gps_read_speed = float(config["gpsSpeed"])
        self.baro_read_speed = float(config["baroSpeed"])

        self.max_ignition_delay = float(config["max_motor_ignition_delay"])

        self.mass = float(config["rocket_mass"])
        self.drag_area = float(config["drag_area"])
        self.drag_coeff = float(config["drag_coeff"])
        self.tvc_noise = float(config["tvc_noise"])
        self.tvc_servo_speed = float(config["tvc_servo_speed"])
        self.linkage_ratio = float(config["tvc_linkage_ratio"])

        self.max_tvc = vector3(0.0, float(
            config["max_tvc_angle"][0]), float(config["max_tvc_angle"][1]))
        self.wind_speed = vector3(float(config["wind_speed"][0]), float(
            config["wind_speed"][1]), float(config["wind_speed"][2]))
        self.mmoi = vector3(float(config["mmoi"][0]), float(
            config["mmoi"][1]), float(config["mmoi"][2]))

        # print(self.motors)
        # print(self.time_step)
        # print(self.simulation_time)
        # print(self.imu_accel_read_speed)
        # print(self.imu_gyro_read_speed)
        # print(self.gps_read_speed)
        # print(self.baro_read_speed)

# pog = settingsParser()

# pog.load_settings("lol.yaml")
