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
        pyrosim.Send_Joint(name='Torso_BackLeg', parent='Torso', child='BackLeg', type='revolute',
                           position=[0, -0.5, 1])
        pyrosim.Send_Joint(name='Torso_FrontLeg', parent='Torso', child='FrontLeg', type='revolute',
                           position=[0, 0.5, 1])
        pyrosim.End()

    def create_brain(self):
        pyrosim.Start_NeuralNetwork(f'brain{self.id}.nndf')
        pyrosim.Send_Sensor_Neuron(name=0, linkName="Torso")
        pyrosim.Send_Sensor_Neuron(name=1, linkName="BackLeg")
        pyrosim.Send_Sensor_Neuron(name=2, linkName="FrontLeg")
        pyrosim.Send_Motor_Neuron(name=3, jointName="Torso_BackLeg")
        pyrosim.Send_Motor_Neuron(name=4, jointName="Torso_FrontLeg")

        for row in range(c.NUM_SENSOR_NEURONS):
            for col in range(c.NUM_MOTOR_NEURONS):
                pyrosim.Send_Synapse(sourceNeuronName=row,
                                     targetNeuronName=col+c.NUM_SENSOR_NEURONS,
                                     weight=self.weights[row][col])
        pyrosim.End()

    def mutate(self):
        idx = random.randint(0, self.weights.size-1)
        new_weight = random.uniform(-1, 1)
        self.weights.ravel()[idx] = new_weight
