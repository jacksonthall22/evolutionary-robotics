import pyrosim.pyrosim as ps
from utils import delete_files, add_tup
import constants as c

try:
    from typing import Literal
except ImportError:
    from utils import Literal
import math
from math import radians
import random
import numpy as np
import os
import sys
import time
import copy
import subprocess


class Solution:
    def __init__(self, id: int, preset_weights: np.ndarray = None):
        self.id = id
        self.weights = preset_weights
        self.fitness = None

    def set_id(self, id):
        self.id = id

    def start_simulation(self, direct_or_gui: Literal['DIRECT', 'GUI']):
        self.create_world()
        self.create_body()
        self.create_brain()
        subprocess.Popen(['nohup', sys.executable, 'simulate.py', direct_or_gui, str(self.id)],
                         stdout=subprocess.DEVNULL,
                         stderr=subprocess.STDOUT)

    def wait_for_simulation_to_end(self):
        while not os.path.exists(self.fitness_filename):
            time.sleep(0.1)
        for i in range(1000):
            try:
                with open(self.fitness_filename, 'r') as f:
                    self.fitness = float(f.read())
                break
            except PermissionError:
                if i == 999:
                    print('test: PermissionError')

        delete_files(file=self.fitness_filename)

    def evaluate(self, direct_or_gui):
        self.start_simulation(direct_or_gui)
        self.wait_for_simulation_to_end()

    X_DISTANCE = 10
    TABLE_SIZE = (1, 1, 1.25)
    TABLE_SIZE_X, TABLE_SIZE_Y, TABLE_SIZE_Z = TABLE_SIZE
    def create_world(self):
        ps.Start_SDF('world.sdf')
        ps.Send_Cube(name='Table',
                     pos=(-Solution.X_DISTANCE, 0, Solution.TABLE_SIZE_Z/2),
                     size=Solution.TABLE_SIZE)
        ps.End()

    def create_body(self):

        ps.Start_URDF('cube.urdf')
        CUBE_SIZE = (0.1, 0.1, 0.1)
        CUBE_SIZE_X, CUBE_SIZE_Y, CUBE_SIZE_Z = CUBE_SIZE
        ps.Send_Cube(name='Cube',
                     pos=(-Solution.X_DISTANCE, 0, Solution.TABLE_SIZE_Z + CUBE_SIZE_Z / 2),
                     size=CUBE_SIZE)
        ps.End()

        ps.Start_URDF('body.urdf')

        # Torso
        TORSO_SIZE = (1.5, 0.5, 0.2)
        TORSO_SIZE_X, TORSO_SIZE_Y, TORSO_SIZE_Z = TORSO_SIZE
        TORSO_POS = (0, 0, 1)
        ps.Send_Cube(name='Torso', pos=TORSO_POS, size=TORSO_SIZE)

        # Hip Joints
        LEG_SIZE = (0.15, 0.1, 0.75)
        LEG_SIZE_X, LEG_SIZE_Y, LEG_SIZE_Z = LEG_SIZE

        HIP_SIZE = (LEG_SIZE[0], 0, LEG_SIZE[0])
        HIP_JOINT_BR = add_tup(TORSO_POS, (+TORSO_SIZE_X/2 - LEG_SIZE_X/2,
                                           +TORSO_SIZE_Y/2,
                                           0))
        HIP_JOINT_BL = add_tup(TORSO_POS, (+TORSO_SIZE_X/2 - LEG_SIZE_X/2,
                                           -TORSO_SIZE_Y/2,
                                           0))
        HIP_JOINT_FR = add_tup(TORSO_POS, (-TORSO_SIZE_X/2 + LEG_SIZE_X/2,
                                           +TORSO_SIZE_Y/2,
                                           0))
        HIP_JOINT_FL = add_tup(TORSO_POS, (-TORSO_SIZE_X/2 + LEG_SIZE_X/2,
                                           -TORSO_SIZE_Y/2,
                                           0))
        HIP_UPPER_LIMIT = radians(0)
        HIP_LOWER_LIMIT = -HIP_UPPER_LIMIT
        # HIP_LOWER_LIMIT = HIP_UPPER_LIMIT = 0
        ps.Send_Joint(parent='Torso',
                      child='BackRightHip',
                      type='revolute',
                      position=HIP_JOINT_BR,
                      axis=(1, 0, 0),
                      lower_limit=HIP_LOWER_LIMIT,
                      upper_limit=HIP_UPPER_LIMIT)
        ps.Send_Joint(parent='Torso',
                      child='BackLeftHip',
                      type='revolute',
                      position=HIP_JOINT_BL,
                      axis=(1, 0, 0),
                      lower_limit=HIP_LOWER_LIMIT,
                      upper_limit=HIP_UPPER_LIMIT)
        ps.Send_Joint(parent='Torso',
                      child='FrontRightHip',
                      type='revolute',
                      position=HIP_JOINT_FR,
                      axis=(1, 0, 0),
                      lower_limit=HIP_LOWER_LIMIT,
                      upper_limit=HIP_UPPER_LIMIT)
        ps.Send_Joint(parent='Torso',
                      child='FrontLeftHip',
                      type='revolute',
                      position=HIP_JOINT_FL,
                      axis=(1, 0, 0),
                      lower_limit=HIP_LOWER_LIMIT,
                      upper_limit=HIP_UPPER_LIMIT)

        # Hip Links
        HIP_POS = (0, 0, 0)
        ps.Send_Cube(name='BackRightHip', pos=HIP_POS, size=(0.1, 0.1, 0.1))
        ps.Send_Cube(name='BackLeftHip', pos=HIP_POS, size=HIP_SIZE)
        ps.Send_Cube(name='FrontRightHip', pos=HIP_POS, size=HIP_SIZE)
        ps.Send_Cube(name='FrontLeftHip', pos=HIP_POS, size=HIP_SIZE)

        # Leg Joints
        LEG_INITIAL_ROT = (0, radians(-50), 0)
        LEG_JOINT = (0, 0, 0)
        LEG_LOWER_LIMIT = radians(-91)
        LEG_UPPER_LIMIT = radians(91)
        ps.Send_Joint(parent='BackRightHip',
                      child='BackRightLeg',
                      type='revolute',
                      position=LEG_JOINT,
                      axis=(0, 1, 0),
                      initial_rot=LEG_INITIAL_ROT,
                      lower_limit=LEG_LOWER_LIMIT,
                      upper_limit=LEG_UPPER_LIMIT)
        ps.Send_Joint(parent='BackLeftHip',
                      child='BackLeftLeg',
                      type='revolute',
                      position=LEG_JOINT,
                      axis=(0, 1, 0),
                      initial_rot=LEG_INITIAL_ROT,
                      lower_limit=LEG_LOWER_LIMIT,
                      upper_limit=LEG_UPPER_LIMIT)
        ps.Send_Joint(parent='FrontRightHip',
                      child='FrontRightLeg',
                      type='revolute',
                      position=LEG_JOINT,
                      axis=(0, 1, 0),
                      initial_rot=LEG_INITIAL_ROT,
                      lower_limit=LEG_LOWER_LIMIT,
                      upper_limit=LEG_UPPER_LIMIT)
        ps.Send_Joint(parent='FrontLeftHip',
                      child='FrontLeftLeg',
                      type='revolute',
                      position=LEG_JOINT,
                      axis=(0, 1, 0),
                      initial_rot=LEG_INITIAL_ROT,
                      lower_limit=LEG_LOWER_LIMIT,
                      upper_limit=LEG_UPPER_LIMIT)

        # Leg Links
        LEG_POS_R = (0,
                     +LEG_SIZE_Y/2,
                     -LEG_SIZE_Z/2 + LEG_SIZE_X/2)
        LEG_POS_L = (0,
                     -LEG_SIZE_Y/2,
                     -LEG_SIZE_Z/2 + LEG_SIZE_X/2)
        ps.Send_Cube(name='BackRightLeg', pos=LEG_POS_R, size=LEG_SIZE)
        ps.Send_Cube(name='BackLeftLeg', pos=LEG_POS_L, size=LEG_SIZE)
        ps.Send_Cube(name='FrontRightLeg', pos=LEG_POS_R, size=LEG_SIZE)
        ps.Send_Cube(name='FrontLeftLeg', pos=LEG_POS_L, size=LEG_SIZE)

        # Lower Leg Joints
        LOWER_LEG_SIZE = (0.1, 0.1, 0.75)
        LOWER_LEG_SIZE_X, LOWER_LEG_SIZE_Y, LOWER_LEG_SIZE_Z = LOWER_LEG_SIZE
        LOWER_LEG_INITIAL_ROT = tuple(np.multiply(LEG_INITIAL_ROT, -2))
        # LOWER_LEG_INITIAL_ROT = (0, 0, 0)
        LOWER_LEG_JOINT_R = (0,
                             LEG_SIZE_Y/2,
                             -LEG_SIZE_Z + LEG_SIZE_X/2)
        LOWER_LEG_JOINT_L = (0,
                             -LEG_SIZE_Y/2,
                             -LEG_SIZE_Z + LEG_SIZE_X/2)
        LOWER_LEG_LOWER_LIMIT = radians(14 - 50)
        LOWER_LEG_UPPER_LIMIT = radians(160 - 50)
        ps.Send_Joint(parent='BackRightLeg',
                      child='LowerBackRightLeg',
                      type='revolute',
                      position=LOWER_LEG_JOINT_R,
                      axis=(0, 1, 0),
                      initial_rot=LOWER_LEG_INITIAL_ROT,
                      lower_limit=LOWER_LEG_LOWER_LIMIT,
                      upper_limit=LOWER_LEG_UPPER_LIMIT)
        ps.Send_Joint(parent='BackLeftLeg',
                      child='LowerBackLeftLeg',
                      type='revolute',
                      position=LOWER_LEG_JOINT_L,
                      axis=(0, 1, 0),
                      initial_rot=LOWER_LEG_INITIAL_ROT,
                      lower_limit=LOWER_LEG_LOWER_LIMIT,
                      upper_limit=LOWER_LEG_UPPER_LIMIT)
        ps.Send_Joint(parent='FrontRightLeg',
                      child='LowerFrontRightLeg',
                      type='revolute',
                      position=LOWER_LEG_JOINT_R,
                      axis=(0, 1, 0),
                      initial_rot=LOWER_LEG_INITIAL_ROT,
                      lower_limit=LOWER_LEG_LOWER_LIMIT,
                      upper_limit=LOWER_LEG_UPPER_LIMIT)
        ps.Send_Joint(parent='FrontLeftLeg',
                      child='LowerFrontLeftLeg',
                      type='revolute',
                      position=LOWER_LEG_JOINT_L,
                      axis=(0, 1, 0),
                      initial_rot=LOWER_LEG_INITIAL_ROT,
                      lower_limit=LOWER_LEG_LOWER_LIMIT,
                      upper_limit=LOWER_LEG_UPPER_LIMIT)

        # Lower Leg Links
        LOWER_LEG_POS = (0, 0, -LOWER_LEG_SIZE_Z/2)
        ps.Send_Cube(name='LowerBackRightLeg', pos=LOWER_LEG_POS, size=LOWER_LEG_SIZE)
        ps.Send_Cube(name='LowerBackLeftLeg', pos=LOWER_LEG_POS, size=LOWER_LEG_SIZE)
        ps.Send_Cube(name='LowerFrontRightLeg', pos=LOWER_LEG_POS, size=LOWER_LEG_SIZE)
        ps.Send_Cube(name='LowerFrontLeftLeg', pos=LOWER_LEG_POS, size=LOWER_LEG_SIZE)

        # Arm Base Joint
        ARM_BASE_SIZE = (0.2, 0.2, 0.1)
        ARM_BASE_SIZE_X, ARM_BASE_SIZE_Y, ARM_BASE_SIZE_Z = ARM_BASE_SIZE
        ARM_BASE_LOWER_LIMIT = radians(-10)
        ARB_BASE_UPPER_LIMIT = radians(10)
        ARM_BASE_JOINT = add_tup(TORSO_POS, (-TORSO_SIZE_X/2 + ARM_BASE_SIZE_X/2,
                                             0,
                                             TORSO_SIZE_Z/2))
        ps.Send_Joint(parent='Torso',
                      child='ArmBase',
                      type='revolute',
                      position=ARM_BASE_JOINT,
                      axis=(0, 0, 1),
                      initial_rot=None,
                      lower_limit=ARM_BASE_LOWER_LIMIT,
                      upper_limit=ARB_BASE_UPPER_LIMIT)

        # Arm Base Link
        ARM_BASE_POS = (0, 0, ARM_BASE_SIZE_Z/2)
        ps.Send_Cube(name='ArmBase', pos=ARM_BASE_POS, size=ARM_BASE_SIZE)

        # Arm 1 Joint (segment farthest from hand)
        ARM_1_SIZE = (0.1, 0.1, 0.65)
        ARM_1_SIZE_X, ARM_1_SIZE_Y, ARM_1_SIZE_Z = ARM_1_SIZE
        ARM_1_JOINT = (0,
                       0,
                       ARM_BASE_SIZE_Z)
        ARM_1_INITIAL_ROT = (0, math.pi/3, 0)
        ARM_1_LOWER_LIMIT = radians(-90)
        ARM_1_UPPER_LIMIT = radians(5)
        ps.Send_Joint(parent='ArmBase',
                      child='Arm1',
                      type='revolute',
                      position=ARM_1_JOINT,
                      axis=(0, 1, 0),
                      initial_rot=ARM_1_INITIAL_ROT,
                      lower_limit=ARM_1_LOWER_LIMIT,
                      upper_limit=ARM_1_UPPER_LIMIT)

        # Arm 1 Link
        ARM_1_POS = (0,
                     0,
                     ARM_1_SIZE_Z/2)
        ps.Send_Cube(name='Arm1', pos=ARM_1_POS, size=ARM_1_SIZE)

        # Arm 2 Joint (segment closest to hand)
        ARM_2_SIZE = (0.1, 0.1, 0.65)
        ARM_2_SIZE_X, ARM_2_SIZE_Y, ARM_2_SIZE_Z = ARM_2_SIZE
        ARM_2_JOINT = (0,
                       0,
                       ARM_1_SIZE_Z)
        ARM_2_INITIAL_ROT = tuple(np.multiply(ARM_1_INITIAL_ROT, -2))
        ARM_2_LOWER_LIMIT = radians(-45)
        ARM_2_UPPER_LIMIT = radians(45)
        ps.Send_Joint(parent='Arm1',
                      child='Arm2',
                      type='revolute',
                      position=ARM_2_JOINT,
                      axis=(0, 1, 0),
                      initial_rot=ARM_2_INITIAL_ROT,
                      lower_limit=ARM_2_LOWER_LIMIT,
                      upper_limit=ARM_2_UPPER_LIMIT)

        # Arm 2 Link
        ARM_2_POS = (0,
                     0,
                     ARM_2_SIZE_Z/2)
        ps.Send_Cube(name='Arm2', pos=ARM_2_POS, size=ARM_2_SIZE)

        # Arm 3 Joint
        ARM_3_SIZE = (0.1, 0.1, 0.2)
        ARM_3_SIZE_X, ARM_3_SIZE_Y, ARM_3_SIZE_Z = ARM_3_SIZE
        ARM_3_JOINT = (0,
                       0,
                       ARM_2_SIZE_Z)
        ARM_3_INITIAL_ROT = (0, -0.5, 0)
        ARM_3_LOWER_LIMIT = radians(-45)
        ARM_3_UPPER_LIMIT = radians(45)
        ps.Send_Joint(parent='Arm2',
                      child='Arm3',
                      type='revolute',
                      position=ARM_3_JOINT,
                      axis=(0, 1, 0),
                      initial_rot=ARM_3_INITIAL_ROT,
                      lower_limit=ARM_3_LOWER_LIMIT,
                      upper_limit=ARM_3_UPPER_LIMIT)

        # Arm 3 Joint
        ARM_3_POS = (0, 0, ARM_3_SIZE_Z/2)
        ps.Send_Cube(name='Arm3', pos=ARM_3_POS, size=ARM_3_SIZE)

        # Wrist Joint
        WRIST_SIZE = (0.1, 0.1, 0.05)
        WRIST_SIZE_X, WRIST_SIZE_Y, WRIST_SIZE_Z = WRIST_SIZE
        WRIST_JOINT = (0,
                       0,
                       ARM_3_SIZE_Z)
        WRIST_INITIAL_ROT = (0, 0, math.pi/4)
        ps.Send_Joint(parent='Arm3', child='Wrist', type='revolute',
                           position=WRIST_JOINT, axis=(0, 0, 1), initial_rot=WRIST_INITIAL_ROT)

        # Wrist Link
        WRIST_POS = (0, 0, WRIST_SIZE_Z/2)
        ps.Send_Cube(name='Wrist', pos=WRIST_POS, size=WRIST_SIZE)

        # Grabber 1 Joint
        GRABBER_LOWER_LIMIT = radians(-90)
        GRABBER_UPPER_LIMIT = radians(90)
        GRABBER_1_SIZE = (0.05, 0.05, 0.2)
        GRABBER_1_SIZE_X, GRABBER_1_SIZE_Y, GRABBER_1_SIZE_Z = GRABBER_1_SIZE
        GRABBER_1_JOINT = (WRIST_SIZE_X/2,
                           0,
                           WRIST_SIZE_Z)
        GRABBER_1_INITIAL_ROT = (0, 0, 0)
        ps.Send_Joint(parent='Wrist',
                      child='Grabber1',
                      type='revolute',
                      position=GRABBER_1_JOINT,
                      axis=(0, 1, 0),
                      initial_rot=GRABBER_1_INITIAL_ROT,
                      lower_limit=GRABBER_LOWER_LIMIT,
                      upper_limit=GRABBER_UPPER_LIMIT)

        # Grabber 1 Link
        GRABBER_1_POS = (GRABBER_1_SIZE_X/2,
                         0,
                         GRABBER_1_SIZE_Z/2)
        ps.Send_Cube(name='Grabber1', pos=GRABBER_1_POS, size=GRABBER_1_SIZE)

        # Grabber 2 Joint
        GRABBER_2_SIZE = (0.05, 0.05, 0.2)
        GRABBER_2_SIZE_X, GRABBER_2_SIZE_Y, GRABBER_2_SIZE_Z = GRABBER_2_SIZE
        GRABBER_2_JOINT = (-WRIST_SIZE_X / 2,
                           0,
                           WRIST_SIZE_Z)
        GRABBER_2_INITIAL_ROT = tuple(np.multiply(GRABBER_1_INITIAL_ROT, -1))
        ps.Send_Joint(parent='Wrist',
                      child='Grabber2',
                      type='revolute',
                      position=GRABBER_2_JOINT,
                      axis=(0, 1, 0),
                      initial_rot=GRABBER_2_INITIAL_ROT,
                      lower_limit=GRABBER_LOWER_LIMIT,
                      upper_limit=GRABBER_UPPER_LIMIT)

        # Grabber 2 Link
        GRABBER_2_POS = (-GRABBER_2_SIZE_X / 2,
                         0,
                         GRABBER_2_SIZE_Z / 2)
        ps.Send_Cube(name='Grabber2', pos=GRABBER_2_POS, size=GRABBER_2_SIZE)

        ps.End()

    def create_brain(self, filename=None):
        if filename is None:
            filename = self.brain_filename
        else:
            assert filename.endswith('.nndf'), 'filename must be .nndf'

        ps.Start_NeuralNetwork(filename)

        num_sensors = [0]
        def send_touch_sensor(link_name):
            ps.Send_Sensor_Neuron(name=num_sensors[0], linkName=link_name)
            num_sensors[0] += 1
        def send_cpg_sensor(cpg):
            ps.Send_CPG_Neuron(name=num_sensors[0],
                               amplitude=cpg.amplitude,
                               period=cpg.period,
                               offset=cpg.offset,
                               timeSteps=c.TIME_STEPS)
            num_sensors[0] += 1

        num_motors = [0]
        def send_motor(joint_name):
            ps.Send_Motor_Neuron(name=num_motors[0]+num_sensors[0], jointName=joint_name)
            num_motors[0] += 1

        sensor_names = [
            'Torso',

            'BackRightLeg',
            'BackLeftLeg',
            'FrontRightLeg',
            'FrontLeftLeg',

            'LowerBackRightLeg',
            'LowerBackLeftLeg',
            'LowerFrontRightLeg',
            'LowerFrontLeftLeg',

            'Arm1',
            'Arm2',
            # 'Arm3',
            # 'Wrist',
            'Grabber1',
            'Grabber2',
        ]
        # Add touch sensors
        for sensor_name in sensor_names:
            send_touch_sensor(sensor_name)

        # Add cpg sensors
        for cpg in c.CPGS:
            send_cpg_sensor(cpg)

        motor_names = [
            # 'Torso_BackRightHip',
            # 'Torso_BackLeftHip',
            # 'Torso_FrontRightHip',
            # 'Torso_FrontLeftHip',

            'BackRightHip_BackRightLeg',
            'BackLeftHip_BackLeftLeg',
            'FrontRightHip_FrontRightLeg',
            'FrontLeftHip_FrontLeftLeg',

            'BackRightLeg_LowerBackRightLeg',
            'BackLeftLeg_LowerBackLeftLeg',
            'FrontRightLeg_LowerFrontRightLeg',
            'FrontLeftLeg_LowerFrontLeftLeg',

            'ArmBase_Arm1',
            'Arm1_Arm2',
            # 'Arm2_Arm3',
            # 'Arm3_Wrist',
            'Wrist_Grabber1',
            'Wrist_Grabber2',
        ]
        # Add motors
        for motor_name in motor_names:
            send_motor(motor_name)

        # Set weights dynamically so we know how many sensors/motors there are
        if self.weights is not None:
            # Preset weights were provided
            assert self.weights.shape == (num_sensors[0], num_motors[0]), \
                    f'{self.weights.shape} != ({num_sensors[0]}, {num_motors[0]})'
        else:
            self.weights: np.ndarray = np.random.rand(num_sensors[0], num_motors[0]) * 2 - 1

        # Add synapses
        for row in range(num_sensors[0]):
            for col in range(num_motors[0]):
                ps.Send_Synapse(sourceNeuronName=row,
                                     targetNeuronName=col+num_sensors[0],
                                     weight=self.weights[row][col])

        ps.End()

    def mutate(self, num_mutations=1):
        for _ in range(num_mutations):
            idx = random.randint(0, self.weights.size-1)
            new_weight = random.uniform(-1, 1)
            self.weights.ravel()[idx] = new_weight
