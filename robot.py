import pybullet as pyb
import pyrosim.pyrosim as ps
from pyrosim.neuralNetwork import NEURAL_NETWORK
from sensor import Sensor
from motor import Motor
import constants as c


class Robot:
    def __init__(self):
        self.sensors = {}
        self.motors = {}
        self.nn = NEURAL_NETWORK('brain.nndf')

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
            f = c.FREQUENCY
            if joint_name == 'Torso_FrontLeg':
                f *= 2

            self.motors[joint_name] = Motor(joint_name,
                                            amplitude=c.AMPLITUDE,
                                            frequency=f,
                                            phase_offset=c.PHASE_OFFSET)

    def act(self, t):
        for neuron_name in self.nn.Get_Neuron_Names():
            if self.nn.Is_Motor_Neuron(neuron_name):
                joint_name = self.nn.Get_Motor_Neurons_Joint(neuron_name)
                desired_angle = self.nn.Get_Value_Of(neuron_name)
                self.motors[joint_name].set_value(self.robot_id, desired_angle)
                print('test:', neuron_name, joint_name, desired_angle)

        # for motor in self.motors.values():
