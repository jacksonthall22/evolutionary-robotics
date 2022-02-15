import pybullet as p
import time
import pybullet_data
from pyrosim import pyrosim
import numpy as np
import math


def main():
    physicsClient = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.8)
    planeId = p.loadURDF('plane.urdf')
    robotId = p.loadURDF('body.urdf')
    p.loadSDF('world.sdf')

    TIME_STEPS = 1000
    TICKS_PER_SEC = 240

    # Joint angles
    amplitude = math.pi / 4
    frequency = 10
    phase_offset = 0
    TARGET_ANGLES = amplitude * np.sin(frequency * np.linspace(0, 2 * math.pi, TIME_STEPS) + phase_offset)
    np.save('data/TARGET_ANGLES.npy', TARGET_ANGLES)

    backLegSensorValues = np.zeros(TIME_STEPS)
    frontLegSensorValues = np.zeros(TIME_STEPS)
    pyrosim.Prepare_To_Simulate(robotId)
    for i in range(TIME_STEPS):
        p.stepSimulation()
        backLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("BackLeg")
        frontLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("FrontLeg")

        pyrosim.Set_Motor_For_Joint(bodyIndex=robotId,
                                    jointName='Torso_BackLeg',
                                    controlMode=p.POSITION_CONTROL,
                                    targetPosition=TARGET_ANGLES[i],
                                    maxForce=50)
        pyrosim.Set_Motor_For_Joint(bodyIndex=robotId,
                                    jointName='Torso_FrontLeg',
                                    controlMode=p.POSITION_CONTROL,
                                    targetPosition=TARGET_ANGLES[i],
                                    maxForce=50)

        print(backLegSensorValues[i])
        time.sleep(1/TICKS_PER_SEC)

    np.save('data/backLegSensorValues.npy', backLegSensorValues)
    np.save('data/frontLegSensorValues.npy', frontLegSensorValues)
    p.disconnect()


if __name__ == '__main__':
    main()
