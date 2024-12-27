import numpy as np

class SE2Transform:
    @staticmethod
    def xy(x, y):
        return np.array([
            [1, 0, x],
            [0, 1, y],
            [0, 0, 1]
        ])

    @staticmethod
    def theta(theta):
        return np.array([
            [np.cos(theta), -np.sin(theta), 0],
            [np.sin(theta),  np.cos(theta), 0],
            [0,              0,             1]
        ])