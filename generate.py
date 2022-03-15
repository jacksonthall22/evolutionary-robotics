import pyrosim.pyrosim as pyrosim
import random


def create_world():
    pyrosim.Start_SDF('world.sdf')
    pyrosim.Send_Cube(name='Box', pos=[2, 2, 0.5], size=[1, 1, 1])
    pyrosim.End()

def generate_body():
    pyrosim.Start_URDF('body.urdf')
    pyrosim.Send_Cube(name='Torso', pos=[0, 0, 1.5], size=[1, 1, 1])
    pyrosim.Send_Cube(name='BackLeg', pos=[-0.5, 0, -0.5], size=[1, 1, 1])
    pyrosim.Send_Cube(name='FrontLeg', pos=[0.5, 0, -0.5], size=[1, 1, 1])
    pyrosim.Send_Joint(name='Torso_BackLeg', parent='Torso', child='BackLeg', type='revolute', position=[-0.5, 0, 1])
    pyrosim.Send_Joint(name='Torso_FrontLeg', parent='Torso', child='FrontLeg', type='revolute', position=[0.5, 0, 1])
    pyrosim.End()

def generate_brain():
    pyrosim.Start_NeuralNetwork('brain.nndf')
    pyrosim.Send_Sensor_Neuron(name=0, linkName="Torso")
    pyrosim.Send_Sensor_Neuron(name=1, linkName="BackLeg")
    pyrosim.Send_Sensor_Neuron(name=2, linkName="FrontLeg")
    pyrosim.Send_Motor_Neuron(name=3, jointName="Torso_BackLeg")
    pyrosim.Send_Motor_Neuron(name=4, jointName="Torso_FrontLeg")

    for source in (0, 1, 2):
        for target in (3, 4):
            pyrosim.Send_Synapse(sourceNeuronName=source,
                                 targetNeuronName=target,
                                 weight=random.uniform(-1, 1))
    pyrosim.End()

def main():
    create_world()
    generate_body()
    generate_brain()


if __name__ == '__main__':
    main()
