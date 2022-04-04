import numpy as np

''' Simulation constants '''
TIME_STEPS = 2000
TICKS_PER_SEC = 1000
NUM_GENERATIONS = 5
POPULATION_SIZE = 15

''' Robot constants '''
MAX_JOINT_FORCE = 50

JOINT_MOTOR_RANGE = 0.2

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

NUM_SENSOR_NEURONS = 9
NUM_MOTOR_NEURONS = 8
