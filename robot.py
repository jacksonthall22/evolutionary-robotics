import pybullet as pyb
import pyrosim.pyrosim as ps
from sensor import Sensor
from motor import Motor
import constants as c


class Robot:
    def __init__(self):
        self.sensors = {}
        self.motors = {}

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
        for motor in self.motors.values():
            motor.set_value(self.robot_id, t)
