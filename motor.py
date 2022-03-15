import numpy as np
import constants as c
import pybullet as pb
import pyrosim.pyrosim as ps


class Motor:
    def __init__(self,
                 joint_name,
                 amplitude,
                 frequency,
                 phase_offset):
        self.joint_name = joint_name
        self.amplitude = amplitude
        self.frequency = frequency
        self.phase_offset = phase_offset

        self.motor_values = amplitude * np.sin(frequency
                                        * np.linspace(0, 2*np.pi, c.TIME_STEPS)
                                        + phase_offset)

    def set_value(self, robot_id, desired_angle):
        ps.Set_Motor_For_Joint(bodyIndex=robot_id,
                               jointName=self.joint_name,
                               controlMode=pb.POSITION_CONTROL,
                               targetPosition=desired_angle,
                               maxForce=c.MAX_JOINT_FORCE)
