import pyrosim.pyrosim as pyrosim


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
    pyrosim.End()

def main():
    create_world()
    generate_body()
    generate_brain()


if __name__ == '__main__':
    main()
