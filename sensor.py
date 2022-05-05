import constants as c
import numpy as np
import pyrosim.pyrosim as ps


class Sensor:
    def __init__(self, name, time_steps):
        self.name = name
        self.values = np.zeros(time_steps)

    def set_value(self, t):
        self.values[t] = ps.Get_Touch_Sensor_Value_For_Link(self.name)

    def save_values(self):
        np.save(f'data/{self.name}_sensor_values.npy', self.values)
