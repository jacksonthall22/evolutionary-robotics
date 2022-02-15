import numpy as np
import matplotlib.pyplot as plt

backLegSensorValues = np.load('data/backLegSensorValues.npy')
frontLegSensorValues = np.load('data/frontLegSensorValues.npy')
target_angles = np.load('data/TARGET_ANGLES.npy')

# plt.plot(backLegSensorValues, label='backLeg', linewidth=2)
# plt.plot(frontLegSensorValues, label='frontLeg', linewidth=2)
plt.plot(target_angles)
plt.legend()
plt.show()
