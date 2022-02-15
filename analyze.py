import numpy as np
import matplotlib.pyplot as plt

backLegSensorValues = np.load('data/backLegSensorValues.npy')
frontLegSensorValues = np.load('data/frontLegSensorValues.npy')

plt.plot(backLegSensorValues, label='backLeg', linewidth=2)
plt.plot(frontLegSensorValues, label='frontLeg', linewidth=2)
plt.legend()
plt.show()
