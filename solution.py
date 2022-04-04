import pyrosim.pyrosim as pyrosim
import random
import numpy as np
import os
import time
import constants as c

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
        pyrosim.Send_Cube(name='Box', pos=[2, 2, 0.5], size=[1, 1, 1])
        pyrosim.End()

    def create_body(self):
        pyrosim.Start_URDF('body.urdf')

        pyrosim.Send_Cube(name='Torso', pos=[0, 0, 1], size=[1, 1, 1])

        pyrosim.Send_Cube(name='BackLeg', pos=[0, -0.5, 0], size=[0.2, 1, 0.2])
        pyrosim.Send_Cube(name='FrontLeg', pos=[0, 0.5, 0], size=[0.2, 1, 0.2])
        pyrosim.Send_Cube(name='LeftLeg', pos=[-0.5, 0, 0], size=[1, 0.2, 0.2])
        pyrosim.Send_Cube(name='RightLeg', pos=[0.5, 0, 0], size=[1, 0.2, 0.2])

        pyrosim.Send_Cube(name='LowerBackLeg', pos=[0, 0, -0.5], size=[0.2, 0.2, 1])
        pyrosim.Send_Cube(name='LowerFrontLeg', pos=[0, 0, -0.5], size=[0.2, 0.2, 1])
        pyrosim.Send_Cube(name='LowerLeftLeg', pos=[0, 0, -0.5], size=[0.2, 0.2, 1])
        pyrosim.Send_Cube(name='LowerRightLeg', pos=[0, 0, -0.5], size=[0.2, 0.2, 1])

        pyrosim.Send_Joint(name='Torso_BackLeg', parent='Torso', child='BackLeg', type='revolute',
                           position=[0, -0.5, 1], joint_angle='1 0 0')
        pyrosim.Send_Joint(name='Torso_FrontLeg', parent='Torso', child='FrontLeg', type='revolute',
                           position=[0, 0.5, 1], joint_angle='1 0 0')
        pyrosim.Send_Joint(name='Torso_LeftLeg', parent='Torso', child='LeftLeg', type='revolute',
                           position=[-0.5, 0, 1], joint_angle='0 1 0')
        pyrosim.Send_Joint(name='Torso_RightLeg', parent='Torso', child='RightLeg', type='revolute',
                           position=[0.5, 0, 1], joint_angle='0 1 0')

        pyrosim.Send_Joint(name='BackLeg_LowerBackLeg', parent='BackLeg', child='LowerBackLeg', type='revolute',
                           position=[0, -1, 0], joint_angle='1 0 0')
        pyrosim.Send_Joint(name='FrontLeg_LowerFrontLeg', parent='FrontLeg', child='LowerFrontLeg', type='revolute',
                           position=[0, 1, 0], joint_angle='1 0 0')
        pyrosim.Send_Joint(name='LeftLeg_LowerLeftLeg', parent='LeftLeg', child='LowerLeftLeg', type='revolute',
                           position=[-1, 0, 0], joint_angle='0 1 0')
        pyrosim.Send_Joint(name='RightLeg_LowerRightLeg', parent='RightLeg', child='LowerRightLeg', type='revolute',
                           position=[1, 0, 0], joint_angle='0 1 0')
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
        for sensor_name in sensor_names:
            send_sensor(sensor_name)

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
        for motor_name in motor_names:
            send_motor(motor_name)


        for row in range(num_sensors[0]):
            for col in range(num_motors[0]):
                pyrosim.Send_Synapse(sourceNeuronName=row,
                                     targetNeuronName=col+num_sensors[0],
                                     weight=self.weights[row][col])
        pyrosim.End()

    def mutate(self):
        idx = random.randint(0, self.weights.size-1)
        new_weight = random.uniform(-1, 1)
        self.weights.ravel()[idx] = new_weight
