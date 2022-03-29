import numpy as np

''' Simulation constants '''
TIME_STEPS = 1000
TICKS_PER_SEC = 100
NUM_GENERATIONS = 1
POPULATION_SIZE = 1

''' Robot constants '''
MAX_JOINT_FORCE = 50

AMPLITUDE = np.pi / 6
FREQUENCY = 11
PHASE_OFFSET = 0
# Front joint angles
TARGET_ANGLES_FRONT = AMPLITUDE * np.sin(FREQUENCY
                                         * np.linspace(0, 2*np.pi, TIME_STEPS)
                                         + PHASE_OFFSET)
# Back joint angles
TARGET_ANGLES_BACK = AMPLITUDE * np.sin(FREQUENCY
                                        * np.linspace(0, 2*np.pi, TIME_STEPS)
                                        + PHASE_OFFSET)

NUM_SENSOR_NEURONS = 3
NUM_MOTOR_NEURONS = 2
