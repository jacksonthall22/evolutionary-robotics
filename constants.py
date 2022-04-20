import numpy as np

''' Simulation constants '''
TIME_STEPS = 2000
TICKS_PER_SEC = 1000
NUM_GENERATIONS = 1
POPULATION_SIZE = 1
MAX_CONCURRENT_SIMS = 15

''' Robot constants '''
NUM_SENSOR_NEURONS = 4
NUM_MOTOR_NEURONS = 6
START_NUM_MUTATIONS = 4
END_NUM_MUTATIONS = 1
NUM_MUTATIONS_FROM_SEED = 4
MAX_JOINT_FORCE = 35
JOINT_MOTOR_RANGE = 1

''' Fitness constants '''
CUBE_HEIGHT_SCALE = 1
FORWARD_POS_SCALE = 0
FORWARD_VEL_SCALE = 0
HEIGHT_SCALE = 0
HEIGHT_CONSISTENCY_SCALE = 0
BALANCING_SCALE = 0
UPRIGHT_SCALE = 0
LEG_MOVEMENT_SCALE = 0
LEG_CONSISTENCY_SCALE = 0
LEGS_FITNESS_SCALE = 0
CONTACT_SCALE = 0
