import pyrosim.pyrosim as pyrosim
import random
import numpy as np
import os

class Solution:
    def __init__(self):
        self.weights = np.random.rand(3, 2) * 2 - 1
        self.fitness = None

    def evaluate(self, direct_or_gui):
        self.create_world()
        self.create_body()
        self.create_brain()
        os.system(f'python simulate.py {direct_or_gui}')

        with open('fitness.txt', 'r') as f:
            self.fitness = float(f.read())

    def create_world(self):
        pyrosim.Start_SDF('world.sdf')
        pyrosim.Send_Cube(name='Box', pos=[2, 2, 0.5], size=[1, 1, 1])
        pyrosim.End()

    def create_body(self):
        pyrosim.Start_URDF('body.urdf')
        pyrosim.Send_Cube(name='Torso', pos=[0, 0, 1.5], size=[1, 1, 1])
        pyrosim.Send_Cube(name='BackLeg', pos=[-0.5, 0, -0.5], size=[1, 1, 1])
        pyrosim.Send_Cube(name='FrontLeg', pos=[0.5, 0, -0.5], size=[1, 1, 1])
        pyrosim.Send_Joint(name='Torso_BackLeg', parent='Torso', child='BackLeg', type='revolute',
                           position=[-0.5, 0, 1])
        pyrosim.Send_Joint(name='Torso_FrontLeg', parent='Torso', child='FrontLeg', type='revolute',
                           position=[0.5, 0, 1])
        pyrosim.End()

    def create_brain(self):
        pyrosim.Start_NeuralNetwork('brain.nndf')
        pyrosim.Send_Sensor_Neuron(name=0, linkName="Torso")
        pyrosim.Send_Sensor_Neuron(name=1, linkName="BackLeg")
        pyrosim.Send_Sensor_Neuron(name=2, linkName="FrontLeg")
        pyrosim.Send_Motor_Neuron(name=3, jointName="Torso_BackLeg")
        pyrosim.Send_Motor_Neuron(name=4, jointName="Torso_FrontLeg")

        for row in range(3):
            for col in range(2):
                pyrosim.Send_Synapse(sourceNeuronName=row,
                                     targetNeuronName=col+3,
                                     weight=self.weights[row][col])
        pyrosim.End()

    def mutate(self):
        idx = random.randint(0, self.weights.size-1)
        new_weight = random.uniform(-1, 1)
        self.weights.ravel()[idx] = new_weight
