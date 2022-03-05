import matplotlib.pyplot as plt
import numpy as np

# use mmoar than 1 h matrix depending on what data is ready

class kalman:

    def __init__(self) -> None:

        self.Xk = np.array([[0.0],  # x
                            [0.0],  # x.
                            [0.0]])  # x..

        self.Fk = np.array([[1.0, 0.0, (0.0 ** 2)/2],
                            [0.0, 1.0, 0.0],
                            [0.0, 0.0, 1.0]])

        self.Pk = np.array([[0.0, 0.0, 0.0],
                            [0.0, 0.0, 0.0],
                            [0.0, 0.0, 0.0]])

        self.Qk = np.array([[0.25, 0.0, 0.0],
                            [0.0,  0.0, 0.0],
                            [0.0,  0.0, 5.0]])

        self.Hk = np.array([[1.0, 0.0, 0.0],
                            [0.0, 0.0, 0.0],
                            [0.0, 0.0, 0.0]])

        self.Rk = np.array([[15.0, 0.0, 0.0],
                            [0.0,  5.0, 0.0],
                            [0.0,  0.0, 0.0]])

        self.K = np.array([[0.0, 0.0, 0.0],
                            [0.0, 0.0, 0.0],
                            [0.0, 0.0, 0.0]])

        self.Zk = np.array([[0.1], [0.5], [0.7]])

    def get_kalman_gain(self, Hk):
        self.K = self.Pk * np.transpose(Hk) * np.linalg.inv(Hk * self.Pk * np.transpose(Hk) + self.Rk)

    def update_measurement(self):
        self.Xk = self.Fk * self.Xk + self.K * (self.Zk - self.Hk * self.Xk)
        self.Pk = self.Pk - (self.K * self.Hk *self.Pk)

    def propogate(self, sens, dt):
        B = np.array([[(dt ** 2)/2],
                                [dt],
                                [1]])
        self.Xk = self.Fk * self.Xk + B * sens
        self.Pk = self.Fk * self.Pk * np.transpose(self.Fk) + self.Qk

k: kalman = kalman()
k.propogate(1, 0.1)
k.propogate(1.1, 0.1)
k.propogate(1.01, 0.1)
k.propogate(1.04, 0.1)
k.update_measurement()
k.get_kalman_gain(k.Hk)