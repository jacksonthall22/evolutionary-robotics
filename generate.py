import pyrosim.pyrosim as pyrosim


def create_world():
    pyrosim.Start_SDF('world.sdf')
    pyrosim.Send_Cube(name='Box', pos=[2, 2, 0.5], size=[1, 1, 1])
    pyrosim.End()

def create_robot():
    pyrosim.Start_URDF('body.urdf')
    pyrosim.Send_Cube(name='Link0', pos=[0, 0, 0.5], size=[1, 1, 1])
    pyrosim.Send_Joint(name='Link0_Link1', parent='Link0', child='Link1', type='revolute', position=[0, 0, 1])
    pyrosim.Send_Cube(name='Link1', pos=[0, 0, 0.5], size=[1, 1, 1])
    pyrosim.Send_Joint(name='Link1_Link2', parent='Link1', child='Link2', type='revolute', position=[0, 0, 1])
    pyrosim.Send_Cube(name='Link2', pos=[0, 0, 0.5], size=[1, 1, 1])
    pyrosim.Send_Joint(name='Link2_Link3', parent='Link2', child='Link3', type='revolute', position=[0, 0.5, 0.5])
    pyrosim.Send_Cube(name='Link3', pos=[0, 0.5, 0], size=[1, 1, 1])
    pyrosim.End()

def main():
    create_world()
    create_robot()


if __name__ == '__main__':
    main()
