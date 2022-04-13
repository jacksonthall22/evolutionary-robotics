import pyrosim.pyrosim as pyrosim
import math
import random
import numpy as np
import os
import time
import constants as c
import operator as op

class Solution:
    def __init__(self, id):
        self.id = id
        self.weights = np.random.rand(c.NUM_SENSOR_NEURONS, c.NUM_MOTOR_NEURONS) * 2 - 1
        self.fitness = None

    def set_id(self, id):
        self.id = id

    def start_simulation(self, direct_or_gui):
        self.create_world()
        self.create_body()
        self.create_brain()
        os.system(f'start /B python simulate.py {direct_or_gui} {self.id}')

    def wait_for_simulation_to_end(self):
        fitness_filename = f'fitness{self.id}.txt'

        while not os.path.exists(fitness_filename):
            time.sleep(0.01)
        for _ in range(1000):
            try:
                with open(fitness_filename, 'r') as f:
                    self.fitness = float(f.read())
                break
            except PermissionError:
                pass

        os.system(f'del fitness{self.id}.txt')

    def evaluate(self, direct_or_gui):
        self.start_simulation(direct_or_gui)
        self.wait_for_simulation_to_end()

    def create_world(self):
        pyrosim.Start_SDF('world.sdf')

        X_DISTANCE = 5

        BOX_1_SIZE = (1, 1, 1.25)
        BOX_1_SIZE_X, BOX_1_SIZE_Y, BOX_1_SIZE_Z = BOX_1_SIZE
        pyrosim.Send_Cube(name='Box1', pos=(-X_DISTANCE, 0, BOX_1_SIZE_Z/2), size=BOX_1_SIZE)

        BOX_2_SIZE = (0.1, 0.1, 0.1)
        BOX_2_SIZE_X, BOX_2_SIZE_Y, BOX_2_SIZE_Z = BOX_2_SIZE
        pyrosim.Send_Cube(name='Box2', pos=(-X_DISTANCE, 0, BOX_1_SIZE_Z+BOX_2_SIZE_Z/2 + 0.25), size=BOX_2_SIZE)

        pyrosim.End()

    def create_body(self):
        pyrosim.Start_URDF('body.urdf')

        def add_tup(t1: tuple, t2: tuple) -> tuple:
            return tuple(map(op.add, t1, t2))

        # Torso
        TORSO_SIZE = (1.5, 0.5, 0.2)
        TORSO_SIZE_X, TORSO_SIZE_Y, TORSO_SIZE_Z = TORSO_SIZE
        TORSO_POS = (0, 0, 1)
        pyrosim.Send_Cube(name='Torso', pos=TORSO_POS, size=TORSO_SIZE)

        # Leg Joints
        LEG_SIZE = (0.15, 0.1, 0.75)
        LEG_SIZE_X, LEG_SIZE_Y, LEG_SIZE_Z = LEG_SIZE
        LEG_INITIAL_ANGLE = (0, -math.pi/4, 0)
        LEG_JOINT_BR = add_tup(TORSO_POS, (+TORSO_SIZE_X/2 - LEG_SIZE_X/2,
                                           +TORSO_SIZE_Y/2,
                                           0))
        LEG_JOINT_BL = add_tup(TORSO_POS, (+TORSO_SIZE_X/2 - LEG_SIZE_X/2,
                                           -TORSO_SIZE_Y/2,
                                           0))
        LEG_JOINT_FR = add_tup(TORSO_POS, (-TORSO_SIZE_X/2 + LEG_SIZE_X/2,
                                           +TORSO_SIZE_Y/2,
                                           0))
        LEG_JOINT_FL = add_tup(TORSO_POS, (-TORSO_SIZE_X/2 + LEG_SIZE_X/2,
                                           -TORSO_SIZE_Y/2,
                                           0))
        pyrosim.Send_Joint(parent='Torso', child='BackRightLeg', type='revolute',
                           position=LEG_JOINT_BR, joint_axis=(0, 1, 0), initial_angle=LEG_INITIAL_ANGLE)
        pyrosim.Send_Joint(parent='Torso', child='BackLeftLeg', type='revolute',
                           position=LEG_JOINT_BL, joint_axis=(0, 1, 0), initial_angle=LEG_INITIAL_ANGLE)
        pyrosim.Send_Joint(parent='Torso', child='FrontRightLeg', type='revolute',
                           position=LEG_JOINT_FR, joint_axis=(0, 1, 0), initial_angle=LEG_INITIAL_ANGLE)
        pyrosim.Send_Joint(parent='Torso', child='FrontLeftLeg', type='revolute',
                           position=LEG_JOINT_FL, joint_axis=(0, 1, 0), initial_angle=LEG_INITIAL_ANGLE)

        # Leg Links
        LEG_POS_R = (0,
                     LEG_SIZE_Y/2,
                     -LEG_SIZE_Z/2 + LEG_SIZE_X/2)
        LEG_POS_L = (0,
                     -LEG_SIZE_Y/2,
                     -LEG_SIZE_Z/2 + LEG_SIZE_X/2)
        pyrosim.Send_Cube(name='BackRightLeg', pos=LEG_POS_R, size=LEG_SIZE)
        pyrosim.Send_Cube(name='BackLeftLeg', pos=LEG_POS_L, size=LEG_SIZE)
        pyrosim.Send_Cube(name='FrontRightLeg', pos=LEG_POS_R, size=LEG_SIZE)
        pyrosim.Send_Cube(name='FrontLeftLeg', pos=LEG_POS_L, size=LEG_SIZE)

        # Lower Leg Joints
        LOWER_LEG_SIZE = (0.1, 0.1, 0.75)
        LOWER_LEG_SIZE_X, LOWER_LEG_SIZE_Y, LOWER_LEG_SIZE_Z = LOWER_LEG_SIZE
        LOWER_LEG_INITIAL_ANGLE = tuple(np.multiply(LEG_INITIAL_ANGLE, -2))
        LOWER_LEG_JOINT_R = (0,
                             LEG_SIZE_Y/2,
                             -LEG_SIZE_Z + LEG_SIZE_X/2)
        LOWER_LEG_JOINT_L = (0,
                             -LEG_SIZE_Y/2,
                             -LEG_SIZE_Z + LEG_SIZE_X/2)
        pyrosim.Send_Joint(parent='BackRightLeg', child='LowerBackRightLeg', type='revolute',
                           position=LOWER_LEG_JOINT_R, joint_axis=(0, 1, 0), initial_angle=LOWER_LEG_INITIAL_ANGLE)
        pyrosim.Send_Joint(parent='BackLeftLeg', child='LowerBackLeftLeg', type='revolute',
                           position=LOWER_LEG_JOINT_L, joint_axis=(0, 1, 0), initial_angle=LOWER_LEG_INITIAL_ANGLE)
        pyrosim.Send_Joint(parent='FrontRightLeg', child='LowerFrontRightLeg', type='revolute',
                           position=LOWER_LEG_JOINT_R, joint_axis=(0, 1, 0), initial_angle=LOWER_LEG_INITIAL_ANGLE)
        pyrosim.Send_Joint(parent='FrontLeftLeg', child='LowerFrontLeftLeg', type='revolute',
                           position=LOWER_LEG_JOINT_L, joint_axis=(0, 1, 0), initial_angle=LOWER_LEG_INITIAL_ANGLE)

        # Lower Leg Links
        LOWER_LEG_POS = (0, 0, -LOWER_LEG_SIZE_Z/2)
        pyrosim.Send_Cube(name='LowerBackRightLeg', pos=LOWER_LEG_POS, size=LOWER_LEG_SIZE)
        pyrosim.Send_Cube(name='LowerBackLeftLeg', pos=LOWER_LEG_POS, size=LOWER_LEG_SIZE)
        pyrosim.Send_Cube(name='LowerFrontRightLeg', pos=LOWER_LEG_POS, size=LOWER_LEG_SIZE)
        pyrosim.Send_Cube(name='LowerFrontLeftLeg', pos=LOWER_LEG_POS, size=LOWER_LEG_SIZE)

        # Arm Base Joint
        ARM_BASE_SIZE = (0.2, 0.2, 0.1)
        ARM_BASE_SIZE_X, ARM_BASE_SIZE_Y, ARM_BASE_SIZE_Z = ARM_BASE_SIZE
        ARM_BASE_JOINT = add_tup(TORSO_POS, (-TORSO_SIZE_X/2 + ARM_BASE_SIZE_X/2,
                                             0,
                                             TORSO_SIZE_Z/2))
        pyrosim.Send_Joint(parent='Torso', child='ArmBase', type='revolute',
                           position=ARM_BASE_JOINT, joint_axis=(0, 0, 1), initial_angle=None)

        # Arm Base Link
        ARM_BASE_POS = (0, 0, ARM_BASE_SIZE_Z/2)
        pyrosim.Send_Cube(name='ArmBase', pos=ARM_BASE_POS, size=ARM_BASE_SIZE)

        # Arm 1 Joint (segment farthest from hand)
        ARM_1_SIZE = (0.1, 0.1, 0.65)
        ARM_1_SIZE_X, ARM_1_SIZE_Y, ARM_1_SIZE_Z = ARM_1_SIZE
        ARM_1_JOINT = (0,
                       0,
                       ARM_BASE_SIZE_Z)
        ARM_1_INITIAL_ANGLE = (0, math.pi/3, 0)
        pyrosim.Send_Joint(parent='ArmBase', child='Arm1', type='revolute',
                           position=ARM_1_JOINT, joint_axis=(0, 1, 0), initial_angle=ARM_1_INITIAL_ANGLE)

        # Arm 1 Link
        ARM_1_POS = (0,
                     0,
                     ARM_1_SIZE_Z/2)
        pyrosim.Send_Cube(name='Arm1', pos=ARM_1_POS, size=ARM_1_SIZE)

        # Arm 2 Joint (segment closest to hand)
        ARM_2_SIZE = (0.1, 0.1, 0.65)
        ARM_2_SIZE_X, ARM_2_SIZE_Y, ARM_2_SIZE_Z = ARM_2_SIZE
        ARM_2_JOINT = (0,
                       0,
                       ARM_1_SIZE_Z)
        ARM_2_INITIAL_ANGLE = tuple(np.multiply(ARM_1_INITIAL_ANGLE, -2))
        pyrosim.Send_Joint(parent='Arm1', child='Arm2', type='revolute',
                           position=ARM_2_JOINT, joint_axis=(0, 1, 0), initial_angle=ARM_2_INITIAL_ANGLE)

        # Arm 2 Link
        ARM_2_POS = (0,
                     0,
                     ARM_2_SIZE_Z/2)
        pyrosim.Send_Cube(name='Arm2', pos=ARM_2_POS, size=ARM_2_SIZE)

        # Arm 3 Joint
        ARM_3_SIZE = (0.1, 0.1, 0.2)
        ARM_3_SIZE_X, ARM_3_SIZE_Y, ARM_3_SIZE_Z = ARM_3_SIZE
        ARM_3_JOINT = (0,
                       0,
                       ARM_2_SIZE_Z)
        ARM_3_INITIAL_ANGLE = (0, -0.5, 0)
        pyrosim.Send_Joint(parent='Arm2', child='Arm3', type='revolute',
                           position=ARM_3_JOINT, joint_axis=(0, 1, 0), initial_angle=ARM_3_INITIAL_ANGLE)

        # Arm 3 Joint
        ARM_3_POS = (0, 0, ARM_3_SIZE_Z/2)
        pyrosim.Send_Cube(name='Arm3', pos=ARM_3_POS, size=ARM_3_SIZE)

        # Wrist Joint
        WRIST_SIZE = (0.1, 0.1, 0.05)
        WRIST_SIZE_X, WRIST_SIZE_Y, WRIST_SIZE_Z = WRIST_SIZE
        WRIST_JOINT = (0,
                       0,
                       ARM_3_SIZE_Z)
        WRIST_INITIAL_ANGLE = (0, 0, math.pi/4)
        pyrosim.Send_Joint(parent='Arm3', child='Wrist', type='revolute',
                           position=WRIST_JOINT, joint_axis=(0, 0, 1), initial_angle=WRIST_INITIAL_ANGLE)

        # Wrist Link
        WRIST_POS = (0, 0, WRIST_SIZE_Z/2)
        pyrosim.Send_Cube(name='Wrist', pos=WRIST_POS, size=WRIST_SIZE)

        # Grabber 1 Joint
        GRABBER_1_SIZE = (0.05, 0.05, 0.2)
        GRABBER_1_SIZE_X, GRABBER_1_SIZE_Y, GRABBER_1_SIZE_Z = GRABBER_1_SIZE
        GRABBER_1_JOINT = (WRIST_SIZE_X/2,
                           0,
                           WRIST_SIZE_Z)
        GRABBER_1_INITIAL_ANGLE = (0, 0, 0)
        pyrosim.Send_Joint(parent='Wrist', child='Grabber1', type='revolute',
                           position=GRABBER_1_JOINT, joint_axis=(0, 1, 0), initial_angle=GRABBER_1_INITIAL_ANGLE)

        # Grabber 1 Link
        GRABBER_1_POS = (GRABBER_1_SIZE_X/2,
                         0,
                         GRABBER_1_SIZE_Z/2)
        pyrosim.Send_Cube(name='Grabber1', pos=GRABBER_1_POS, size=GRABBER_1_SIZE)

        # Grabber 2 Joint
        GRABBER_2_SIZE = (0.05, 0.05, 0.2)
        GRABBER_2_SIZE_X, GRABBER_2_SIZE_Y, GRABBER_2_SIZE_Z = GRABBER_2_SIZE
        GRABBER_2_JOINT = (-WRIST_SIZE_X / 2,
                           0,
                           WRIST_SIZE_Z)
        GRABBER_2_INITIAL_ANGLE = tuple(np.multiply(GRABBER_1_INITIAL_ANGLE, -1))
        pyrosim.Send_Joint(parent='Wrist', child='Grabber2', type='revolute',
                           position=GRABBER_2_JOINT, joint_axis=(0, 1, 0), initial_angle=GRABBER_2_INITIAL_ANGLE)

        # Grabber 2 Link
        GRABBER_2_POS = (-GRABBER_2_SIZE_X / 2,
                         0,
                         GRABBER_2_SIZE_Z / 2)
        pyrosim.Send_Cube(name='Grabber2', pos=GRABBER_2_POS, size=GRABBER_2_SIZE)

        pyrosim.End()

    def create_brain(self, filename=None):
        if filename is None:
            filename = f'brain{self.id}.nndf'
        else:
            assert filename.endswith('.nndf'), 'filename must be .nndf'

        pyrosim.Start_NeuralNetwork(filename)

        num_sensors = [0]
        def send_sensor(link_name):
            pyrosim.Send_Sensor_Neuron(name=num_sensors[0], linkName=link_name)
            num_sensors[0] += 1

        num_motors = [0]
        def send_motor(joint_name):
            pyrosim.Send_Motor_Neuron(name=num_motors[0]+num_sensors[0], jointName=joint_name)
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
        ]
        for sensor_name in sensor_names:
            send_sensor(sensor_name)

        motor_names = [
            'Torso_BackRightLeg',
            'Torso_BackLeftLeg',
            'Torso_FrontRightLeg',
            'Torso_FrontLeftLeg',

            'BackRightLeg_LowerBackRightLeg',
            'BackLeftLeg_LowerBackLeftLeg',
            'FrontRightLeg_LowerFrontRightLeg',
            'FrontLeftLeg_LowerFrontLeftLeg',
        ]
        for motor_name in motor_names:
            send_motor(motor_name)

        for row in range(num_sensors[0]):
            for col in range(num_motors[0]):
                pyrosim.Send_Synapse(sourceNeuronName=row,
                                     targetNeuronName=col+num_sensors[0],
                                     weight=self.weights[row][col])

        pyrosim.End()

    def mutate(self):
        for _ in range(c.NUM_MUTATIONS):
            idx = random.randint(0, self.weights.size-1)
            new_weight = random.uniform(-1, 1)
            self.weights.ravel()[idx] = new_weight
