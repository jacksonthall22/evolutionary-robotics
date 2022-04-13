import pybullet as pyb
import pyrosim.pyrosim as ps
from pyrosim.neuralNetwork import NEURAL_NETWORK
from sensor import Sensor
from motor import Motor
import constants as c
import os


class Robot:
    def __init__(self, id, brain_filename=None):
        if brain_filename is None:
            brain_filename = f'brain{id}.nndf'

        self.id = id
        self.sensors = {}
        self.motors = {}
        self.nn = NEURAL_NETWORK(brain_filename)
        os.system(f'del brain{id}.nndf')

        self.robot_id = pyb.loadURDF('body.urdf')
        ps.Prepare_To_Simulate(self.robot_id)
        self.prepare_to_sense()
        self.prepare_to_act()

    def prepare_to_sense(self):
        self.sensors = {}
        for link_name in ps.linkNamesToIndices:
            self.sensors[link_name] = Sensor(link_name)

    def sense(self, t):
        for sensor in self.sensors.values():
            sensor.get_value(t)

    def think(self, t):
        # pass
        self.nn.Update()

    def prepare_to_act(self):
        self.motors = {}
        for joint_name in ps.jointNamesToIndices:
            self.motors[joint_name] = Motor(joint_name,
                                            amplitude=c.AMPLITUDE,
                                            frequency=c.FREQUENCY,
                                            phase_offset=c.PHASE_OFFSET)

    def act(self, t):
        for neuron_name in self.nn.Get_Neuron_Names():
            if self.nn.Is_Motor_Neuron(neuron_name):
                joint_name = self.nn.Get_Motor_Neurons_Joint(neuron_name)
                desired_angle = self.nn.Get_Value_Of(neuron_name)
                self.motors[joint_name].set_value(self.robot_id, desired_angle*c.JOINT_MOTOR_RANGE)
                # print('test:', neuron_name, joint_name, desired_angle)

    def get_fitness(self):
        x_coord_of_link_0 = pyb.getLinkState(self.robot_id, 0)[0][0]

        os.system(f'del fitness{self.id}.txt')
        with open(f'tmp{self.id}.txt', 'w') as f:
            f.write(str(x_coord_of_link_0))
        os.system(f'rename tmp{self.id}.txt fitness{self.id}.txt')
