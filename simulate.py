from simulation import Simulation


simulation = Simulation()
simulation.run()


# import pybullet as p
# import time
# import pybullet_data
# from pyrosim import pyrosim
# import numpy as np
# import math
# import constants as c
#
#
# def main():
#     physicsClient = p.connect(p.GUI)
#     p.setAdditionalSearchPath(pybullet_data.getDataPath())
#     p.setGravity(0, 0, -9.8)
#     planeId = p.loadURDF('plane.urdf')
#     robotId = p.loadURDF('body.urdf')
#     p.loadSDF('world.sdf')
#
#     # np.save('data/TARGET_ANGLES_FRONT.npy', c.TARGET_ANGLES_FRONT)
#     # np.save('data/TARGET_ANGLES_BACK.npy', c.TARGET_ANGLES_BACK)
#
#     backLegSensorValues = np.zeros(c.TIME_STEPS)
#     frontLegSensorValues = np.zeros(c.TIME_STEPS)
#
#     pyrosim.Prepare_To_Simulate(robotId)
#     for i in range(c.TIME_STEPS):
#         p.stepSimulation()
#
#         pyrosim.Set_Motor_For_Joint(bodyIndex=robotId,
#                                     jointName='Torso_BackLeg',
#                                     controlMode=p.POSITION_CONTROL,
#                                     targetPosition=c.TARGET_ANGLES_BACK[i],
#                                     maxForce=c.MAX_JOINT_FORCE)
#         pyrosim.Set_Motor_For_Joint(bodyIndex=robotId,
#                                     jointName='Torso_FrontLeg',
#                                     controlMode=p.POSITION_CONTROL,
#                                     targetPosition=c.TARGET_ANGLES_FRONT[i],
#                                     maxForce=c.MAX_JOINT_FORCE)
#
#         backLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("BackLeg")
#         frontLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("FrontLeg")
#         print(backLegSensorValues[i])
#
#         time.sleep(1/c.TICKS_PER_SEC)
#
#     np.save('data/backLegSensorValues.npy', backLegSensorValues)
#     np.save('data/frontLegSensorValues.npy', frontLegSensorValues)
#
#     p.disconnect()
#
#
# if __name__ == '__main__':
#     main()
