import pyrosim.pyrosim as pyrosim


def main():
    pyrosim.Start_SDF('world.sdf')
    pyrosim.Send_Cube(name='Box', pos=[0, 0, 0.5], size=[1, 1, 1])
    pyrosim.End()


if __name__ == '__main__':
    main()
