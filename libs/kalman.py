from cv2 import KalmanFilter
import matplotlib.pyplot as plt
import numpy as np

# use mmoar than 1 h matrix depending on what data is ready

class altitudeKF:

    def __init__(self):

        self.x = np.array([ [0.0],
                            [0.0],
                            [0.0]])

        self.Pk = np.array([[0, 0, 0],
                            [0, 0, 0],
                            [0, 0, 0]])

        self.Qk = np.array([ [1.0, 0, 0],
                             [0, 0.1, 0],
                             [0, 0, 0.1]])

        self.HkAll = np.array([ [1, 0, 0],
                                [0, 0, 0],
                                [0, 0, 1]])

        self.HkBaro = np.array([    [1, 0, 0],
                                    [0, 0, 0],
                                    [0, 0, 0]])

        self.HkAccel = np.array([   [0, 0, 0],
                                    [0, 0, 0],
                                    [0, 0, 1]])

        self.Rk = np.array([[10.0, 0, 0],
                            [0, 1, 0],
                            [0, 0, 0.1]])

    
    def Fk(self, dt):

        F = np.array([[1, dt, (dt*dt)/2],
                    [0, 1, dt],
                    [0, 0, 1]])

        return F

    def Zk(self, pos, acc):

        Z = np.array([  [pos],
                        [0.0],
                        [acc]])

        return Z

    def K(self, Hk):

        self.Kk = self.Pk*np.transpose(Hk) * np.linalg.inv((Hk*self.Pk*np.transpose(Hk) + self.Rk))

    def update(self, Hk, pos, acc):

        self.x = self.x + self.Kk*(self.Zk(pos, acc) - Hk*self.x)
        self.Pk = self.Pk - self.Kk*Hk*self.Pk

    def predict(self, dt):

        self.x = self.Fk(dt)*self.x
        self.Pk = self.Fk(dt)*self.Pk*np.transpose(self.Fk(dt)) + self.Qk

# def K(Pk, Hk, Rk):

#     K = Pk*np.transpose(Hk) * np.linalg.inv((Hk*Pk*np.transpose(Hk) + Rk))

#     return K

# def update(x, Hk, Pk, K, Zk):

#     x = x + K*(Zk - Hk*x)
#     Pk = Pk - K*Hk*Pk

#     return x, Pk

# def predict(x, Fk, Pk, Qk):

#     x = Fk*x
#     Pk = Fk*Pk*np.transpose(Fk) + Qk

#     return x, Pk

position_real = [0]
position_estimate = [0]

velocity_real = [0]
velocity_estimate = [0]

acceleration_real = [0]
acceleration_estimate = [0]

PPlot = [0]
KPlot = [0]

time = 0.0
dt= 0.01
i = 0

baropos = 0.0

KF = altitudeKF()

baropos = np.random.normal(0.0, 1, 1)[0]
KF.K(KF.HkAll)
KF.predict(dt)
KF.update(KF.HkAll, baropos, np.random.normal(0.0, 0.05, 1)[0])
while True:
    i += 1
    time += dt

    accel = 0.0

    vel = velocity_real[-1] + accel*dt
    pos = position_real[-1] + vel*dt

    position_real.append(pos)
    velocity_real.append(vel)
    acceleration_real.append(accel)

    # if i % 10 == 0:
    baropos = np.random.normal(pos, 0.2, 1)[0]
    KF.K(KF.HkAll)
    KF.predict(dt)
    KF.update(KF.HkAll, baropos, np.random.normal(accel, 0.1, 1)[0])

    # else:
        # KF.K(KF.HkAll)
        # KF.predict(dt)
        # KF.update(KF.HkAccel, 0.0, np.random.normal(accel, 0.05, 1)[0])

        # KF.K(KF.HkAccel)
        # KF.predict(dt)
        # KF.update(KF.HkAccel, 0.0, np.random.normal(accel, 0.05, 1)[0])
        

    print(KF.Pk)
    print("")

    position_estimate.append(KF.x[0][0])
    velocity_estimate.append(KF.x[1][1])
    acceleration_estimate.append(KF.x[2][2])

    KPlot.append(KF.Kk[2][2])
    PPlot.append(KF.Pk[0][0])

    if time > 10.0:
        break

t = np.arange(0.0, time+0.01, dt)

plt.figure(1)
plt.plot(t, position_real, label='real')
plt.plot(t, position_estimate, label='estimate')

plt.figure(2)
plt.plot(t, velocity_real, label='real')
plt.plot(t, velocity_estimate, label='estimate')

plt.figure(3)
plt.plot(t, acceleration_real, label='real')
plt.plot(t, acceleration_estimate, label='estimate')

plt.figure(4)
plt.plot(t, PPlot, label='K')


plt.show()