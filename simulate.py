import pybullet as p
import time
import pybullet_data


def main():
    physicsClient = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.8)
    planeId = p.loadURDF("plane.urdf")
    p.loadSDF("world.sdf")

    for i in range(10000):
        p.stepSimulation()
        time.sleep(1/100)
        print(i)

    p.disconnect()


if __name__ == '__main__':
    main()
