import pyrosim.pyrosim as pyrosim
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
        pyrosim.Send_Cube(name='Box', pos=(0, 2, 0.5), size=(1, 1, 1))
        pyrosim.End()

    def create_body(self):
        pyrosim.Start_URDF('body.urdf')

        def add_tup(t1: tuple, t2: tuple) -> tuple:
            return tuple(map(op.add, t1, t2))

        def sub_tup(t1: tuple, t2: tuple) -> tuple:
            return tuple(map(op.sub, t1, t2))

        # Torso
        TORSO_SIZE = (1.5, 0.5, 0.2)
        TORSO_SIZE_X, TORSO_SIZE_Y, TORSO_SIZE_Z = TORSO_SIZE
        TORSO_POS = (0, 0, 2)
        TORSO_POS_X, TORSO_POS_Y, TORSO_POS_Z = TORSO_POS
        pyrosim.Send_Cube(name='Torso', pos=TORSO_POS, size=TORSO_SIZE)

        # Leg Joints
        LEG_SIZE = (0.15, 0.1, 0.75)
        LEG_SIZE_X, LEG_SIZE_Y, LEG_SIZE_Z = LEG_SIZE
        LEG_INITIAL_ANGLE = (0, -0.75, 0)
        LEG_JOINT_BR = add_tup(TORSO_POS, (-TORSO_POS_X + TORSO_SIZE_X/2 - LEG_SIZE_X/2,
                                           -TORSO_POS_Y + TORSO_SIZE_Y/2 + LEG_SIZE_Y/2,
                                           0))
        LEG_JOINT_BL = add_tup(TORSO_POS, (-TORSO_POS_X + TORSO_SIZE_X/2 - LEG_SIZE_X/2,
                                           +TORSO_POS_Y - TORSO_SIZE_Y/2 - LEG_SIZE_Y/2,
                                           0))
        LEG_JOINT_FR = add_tup(TORSO_POS, (+TORSO_POS_X - TORSO_SIZE_X/2 + LEG_SIZE_X/2,
                                           -TORSO_POS_Y + TORSO_SIZE_Y/2 + LEG_SIZE_Y/2,
                                           0))
        LEG_JOINT_FL = add_tup(TORSO_POS, (+TORSO_POS_X - TORSO_SIZE_X/2 + LEG_SIZE_X/2,
                                           +TORSO_POS_Y - TORSO_SIZE_Y/2 - LEG_SIZE_Y/2,
                                           0))
        pyrosim.Send_Joint(name='Torso_BackRightLeg', parent='Torso', child='BackRightLeg', type='revolute',
                           position=LEG_JOINT_BR, joint_axis=(0, 1, 0), initial_angle=LEG_INITIAL_ANGLE)
        pyrosim.Send_Joint(name='Torso_BackLeftLeg', parent='Torso', child='BackLeftLeg', type='revolute',
                           position=LEG_JOINT_BL, joint_axis=(0, 1, 0), initial_angle=LEG_INITIAL_ANGLE)
        pyrosim.Send_Joint(name='Torso_FrontRightLeg', parent='Torso', child='FrontRightLeg', type='revolute',
                           position=LEG_JOINT_FR, joint_axis=(0, 1, 0), initial_angle=LEG_INITIAL_ANGLE)
        pyrosim.Send_Joint(name='Torso_FrontLeftLeg', parent='Torso', child='FrontLeftLeg', type='revolute',
                           position=LEG_JOINT_FL, joint_axis=(0, 1, 0), initial_angle=LEG_INITIAL_ANGLE)

        # Leg Links
        LEG_POS_BR = (0, 0, -LEG_SIZE_Z/2 + TORSO_SIZE_Z/2)
        LEG_POS_BL = (0, 0, -LEG_SIZE_Z/2 + TORSO_SIZE_Z/2)
        LEG_POS_FR = (0, 0, -LEG_SIZE_Z/2 + TORSO_SIZE_Z/2)
        LEG_POS_FL = (0, 0, -LEG_SIZE_Z/2 + TORSO_SIZE_Z/2)
        pyrosim.Send_Cube(name='BackRightLeg', pos=LEG_POS_BR, size=LEG_SIZE)
        pyrosim.Send_Cube(name='BackLeftLeg', pos=LEG_POS_BL, size=LEG_SIZE)
        pyrosim.Send_Cube(name='FrontRightLeg', pos=LEG_POS_FR, size=LEG_SIZE)
        pyrosim.Send_Cube(name='FrontLeftLeg', pos=LEG_POS_FL, size=LEG_SIZE)

        # Lower Leg Joints
        LOWER_LEG_SIZE = (0.1, 0.1, 0.75)
        LOWER_LEG_SIZE_X, LOWER_LEG_SIZE_Y, LOWER_LEG_SIZE_Z = LOWER_LEG_SIZE
        LOWER_LEG_INITIAL_ANGLE = tuple(np.multiply(LEG_INITIAL_ANGLE, -2))
        LOWER_LEG_JOINT_BR = (0, 0, -LEG_SIZE_Z + TORSO_SIZE_Z/2)
        LOWER_LEG_JOINT_BL = (0, 0, -LEG_SIZE_Z + TORSO_SIZE_Z/2)
        LOWER_LEG_JOINT_FR = (0, 0, -LEG_SIZE_Z + TORSO_SIZE_Z/2)
        LOWER_LEG_JOINT_FL = (0, 0, -LEG_SIZE_Z + TORSO_SIZE_Z/2)
        pyrosim.Send_Joint(name='BackRightLeg_LowerBackRightLeg', parent='BackRightLeg', child='LowerBackRightLeg', type='revolute',
                           position=LOWER_LEG_JOINT_BR, joint_axis=(0, 1, 0), initial_angle=LOWER_LEG_INITIAL_ANGLE)
        pyrosim.Send_Joint(name='BackLeftLeg_LowerBackLeftLeg', parent='BackLeftLeg', child='LowerBackLeftLeg', type='revolute',
                           position=LOWER_LEG_JOINT_BL, joint_axis=(0, 1, 0), initial_angle=LOWER_LEG_INITIAL_ANGLE)
        pyrosim.Send_Joint(name='FrontRightLeg_LowerFrontRightLeg', parent='FrontRightLeg', child='LowerFrontRightLeg', type='revolute',
                           position=LOWER_LEG_JOINT_FR, joint_axis=(0, 1, 0), initial_angle=LOWER_LEG_INITIAL_ANGLE)
        pyrosim.Send_Joint(name='FrontLeftLeg_LowerFrontLeftLeg', parent='FrontLeftLeg', child='LowerFrontLeftLeg', type='revolute',
                           position=LOWER_LEG_JOINT_FL, joint_axis=(0, 1, 0), initial_angle=LOWER_LEG_INITIAL_ANGLE)

        # Lower Leg Links
        LOWER_LEG_POS_BR = (0, 0, -LOWER_LEG_SIZE_Z/2)
        LOWER_LEG_POS_BL = (0, 0, -LOWER_LEG_SIZE_Z/2)
        LOWER_LEG_POS_FR = (0, 0, -LOWER_LEG_SIZE_Z/2)
        LOWER_LEG_POS_FL = (0, 0, -LOWER_LEG_SIZE_Z/2)
        pyrosim.Send_Cube(name='LowerBackRightLeg', pos=LOWER_LEG_POS_BR, size=LOWER_LEG_SIZE)
        pyrosim.Send_Cube(name='LowerBackLeftLeg', pos=LOWER_LEG_POS_BL, size=LOWER_LEG_SIZE)
        pyrosim.Send_Cube(name='LowerFrontRightLeg', pos=LOWER_LEG_POS_FR, size=LOWER_LEG_SIZE)
        pyrosim.Send_Cube(name='LowerFrontLeftLeg', pos=LOWER_LEG_POS_FL, size=LOWER_LEG_SIZE)

        pyrosim.End()

    def create_brain(self):
        pyrosim.Start_NeuralNetwork(f'brain{self.id}.nndf')

        num_sensors = [0]
        def send_sensor(link_name):
            pyrosim.Send_Sensor_Neuron(name=num_sensors[0], linkName=link_name)
            num_sensors[0] += 1

        num_motors = [0]
        def send_motor(joint_name):
            pyrosim.Send_Motor_Neuron(name=num_motors[0]+num_sensors[0], jointName=joint_name)
            num_motors[0] += 1

        sensor_names = [
            # 'Torso',

            # 'BackLeg',
            # 'FrontLeg',
            # 'LeftLeg',
            # 'RightLeg',

            'LowerBackLeg',
            'LowerFrontLeg',
            'LowerLeftLeg',
            'LowerRightLeg',
        ]
        # for sensor_name in sensor_names:
        #     send_sensor(sensor_name)

        motor_names = [
            'Torso_BackLeg',
            'Torso_FrontLeg',
            'Torso_LeftLeg',
            'Torso_RightLeg',

            'BackLeg_LowerBackLeg',
            'FrontLeg_LowerFrontLeg',
            'LeftLeg_LowerLeftLeg',
            'RightLeg_LowerRightLeg',
        ]
        # for motor_name in motor_names:
        #     send_motor(motor_name)


        # for row in range(num_sensors[0]):
        #     for col in range(num_motors[0]):
        #         pyrosim.Send_Synapse(sourceNeuronName=row,
        #                              targetNeuronName=col+num_sensors[0],
        #                              weight=self.weights[row][col])

        pyrosim.End()

    def mutate(self):
        idx = random.randint(0, self.weights.size-1)
        new_weight = random.uniform(-1, 1)
        self.weights.ravel()[idx] = new_weight
