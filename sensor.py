import constants as c
import numpy as np
import pyrosim.pyrosim as ps


class Sensor:
    def __init__(self, link_name):
        self.link_name = link_name
        self.values = np.zeros(c.TIME_STEPS)

    def get_value(self, t):
        self.values[t] = ps.Get_Touch_Sensor_Value_For_Link(self.link_name)

    def save_values(self):
        np.save(f'data/{self.link_name}_sensor_values.npy', self.values)
