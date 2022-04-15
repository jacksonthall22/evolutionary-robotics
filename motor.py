import numpy as np
import constants as c
import pybullet as pb
import pyrosim.pyrosim as ps


class Motor:
    def __init__(self,
                 joint_name):
        self.joint_name = joint_name

    def set_value(self, robot_id, desired_angle):
        ps.Set_Motor_For_Joint(bodyIndex=robot_id,
                               jointName=self.joint_name,
                               controlMode=pb.POSITION_CONTROL,
                               targetPosition=desired_angle,
                               maxForce=c.MAX_JOINT_FORCE)
