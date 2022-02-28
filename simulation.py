from world import World
from robot import Robot
import pybullet as pb
import pybullet_data
import pyrosim.pyrosim as ps
import constants as c
import time

class Simulation:
    def __init__(self):
        self.physics_client = pb.connect(pb.GUI)
        pb.setAdditionalSearchPath(pybullet_data.getDataPath())
        pb.setGravity(0, 0, -9.8)

        self.world = World()
        self.robot = Robot()

        ps.Prepare_To_Simulate(self.robot.robot_id)

    def __del__(self):
        pb.disconnect()

    def run(self):
        for t in range(c.TIME_STEPS):
            # print(t)
            pb.stepSimulation()

            self.robot.sense(t)
            self.robot.think(t)
            self.robot.act(t)

            time.sleep(1/c.TICKS_PER_SEC)
