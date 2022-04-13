from world import World
from robot import Robot
import pybullet as pb
import pybullet_data
import pyrosim.pyrosim as ps
import constants as c
import time

class Simulation:
    def __init__(self, direct_or_gui, id, brain_filename=None):
        if direct_or_gui not in ('DIRECT', 'GUI'):
            raise ValueError
        direct_or_gui = (pb.DIRECT, pb.GUI)[direct_or_gui == 'GUI']
        self.direct_or_gui = direct_or_gui

        self.physics_client = pb.connect(direct_or_gui)
        pb.setAdditionalSearchPath(pybullet_data.getDataPath())
        pb.setGravity(0, 0, -9.8)

        self.world = World()
        self.robot = Robot(id, brain_filename=brain_filename)

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

            if self.direct_or_gui == pb.GUI:
                time.sleep(1/c.TICKS_PER_SEC)

    def get_fitness(self):
        self.robot.get_fitness()
