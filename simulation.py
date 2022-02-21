from world import World
from robot import Robot
import pybullet as pyb
from pyrosim import pyrosim as ps


class Simulation:
    def __init__(self):
        self.world = World()
        self.robot = Robot()
        self.physics_client = pyb.connect(pyb.GUI)

        pyb.setAdditionalSearchPath(pybullet_data.getDataPath())
        pyb.setGravity(0, 0, -9.8)
        ps.Prepare_To_Simulate(self.robot.robot_id)
