import random
from sensor import Sensor
from math import sin, pi


class CPG(Sensor):
    name = 'CPG{}'
    __total_num_cpgs = 0

    def __init__(self,
                 time_steps: int,
                 name: str = None,
                 amplitude: float = 50,
                 period: float = pi,
                 offset: float = None):
        if name is None:
            name = CPG.name.format(CPG.__total_num_cpgs)
            CPG.__total_num_cpgs += 1
        super(CPG, self).__init__(name=name, time_steps=time_steps)

        if offset is None:
            offset = random.random() * period

        self.amplitude = amplitude
        self.period = period
        self.offset = offset

        # No need to calculate self.values dynamically
        # https://www.desmos.com/calculator/s5hb7vdc98
        self.values = [CPG.get_value_static(amplitude, period, offset, t) for t in range(time_steps)]

    def set_value(self, t):
        # self.values is precalculated
        pass

    @staticmethod
    def get_value_static(a, p, o, t):
        return a * sin(2 * pi * (t - o) / p)
