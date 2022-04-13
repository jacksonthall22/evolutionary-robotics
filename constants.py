import numpy as np

''' Simulation constants '''
TIME_STEPS = 2000
TICKS_PER_SEC = 1000
NUM_GENERATIONS = 1
POPULATION_SIZE = 1

''' Robot constants '''
NUM_SENSOR_NEURONS = 5
NUM_MOTOR_NEURONS = 8
NUM_MUTATIONS = 2

MAX_JOINT_FORCE = 40

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

