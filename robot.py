import pybullet as pyb


class Robot:
    def __init__(self):
        self.robot_id = pyb.loadURDF('body.urdf')

        self.sensors = {}
        self.motors = {}
