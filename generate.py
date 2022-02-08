import pyrosim.pyrosim as pyrosim


def create_world():
    pyrosim.Start_SDF('world.sdf')
    pyrosim.Send_Cube(name='Box', pos=[2, 2, 0.5], size=[1, 1, 1])
    pyrosim.End()

def create_robot():
    pyrosim.Start_URDF('body.urdf')
    pyrosim.Send_Cube(name='Torso', pos=[0, 0, 1.5], size=[1, 1, 1])
    pyrosim.Send_Cube(name='Backleg', pos=[-0.5, 0, -0.5], size=[1, 1, 1])
    pyrosim.Send_Cube(name='Frontleg', pos=[0.5, 0, -0.5], size=[1, 1, 1])
    pyrosim.Send_Joint(name='Torso_Backleg', parent='Torso', child='Backleg', type='revolute', position=[-0.5, 0, 1])
    pyrosim.Send_Joint(name='Torso_Frontleg', parent='Torso', child='Frontleg', type='revolute', position=[0.5, 0, 1])
    pyrosim.End()

def main():
    create_world()
    create_robot()


if __name__ == '__main__':
    main()
