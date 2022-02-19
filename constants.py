import numpy as np

''' Simulation constants '''
TIME_STEPS = 1000
TICKS_PER_SEC = 240

''' Joint constants '''
MAX_JOINT_FORCE = 50

# Front joint angles
amplitude = np.pi / 6
frequency = 11
phase_offset = 0
TARGET_ANGLES_FRONT = amplitude * np.sin(frequency
                                         * np.linspace(0, 2*np.pi, TIME_STEPS)
                                         + phase_offset)
# Back joint angles
amplitude = np.pi / 6
frequency = 11
phase_offset = 0
TARGET_ANGLES_BACK = amplitude * np.sin(frequency
                                        * np.linspace(0, 2*np.pi, TIME_STEPS)
                                        + phase_offset)
