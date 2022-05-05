from enum import Enum, auto
from CPG import CPG

class MutationType(Enum):
    CONSTANT = auto()
    LINEAR = auto()
    DECAY = auto()
    NEGATIVE_EXPONENTIAL = auto()


''' Simulation constants '''
TIME_STEPS = 2000
TICKS_PER_SEC = 1000
NUM_GENERATIONS = 15
POPULATION_SIZE = 10
MAX_CONCURRENT_SIMS = 10

''' Mutation constants '''
START_NUM_MUTATIONS = 50
END_NUM_MUTATIONS = 3
MUTATION_TYPE = MutationType.CONSTANT
NUM_MUTATIONS_FROM_SEED = 4

''' Robot constants '''
MAX_JOINT_FORCE = 35
JOINT_MOTOR_RANGE = 1
CPGS = [
    # CPG(time_steps=TIME_STEPS, period=16),
    # CPG(time_steps=TIME_STEPS, period=32),
    CPG(time_steps=TIME_STEPS, period=64),
    CPG(time_steps=TIME_STEPS, period=128),
    CPG(time_steps=TIME_STEPS, period=256),
    # CPG(time_steps=TIME_STEPS, period=512),
    # CPG(time_steps=TIME_STEPS, period=1024),
]

''' Fitness constants '''
CUBE_HEIGHT_SCALE = 0
GRABBER_DIST_SCALE = 0
FORWARD_POS_SCALE = 1
FORWARD_VEL_SCALE = 0
HEIGHT_SCALE = 0
HEIGHT_CONSISTENCY_SCALE = 0
BALANCING_SCALE = 0
UPRIGHT_SCALE = 0
LEG_MOVEMENT_SCALE = 0
LEG_CONSISTENCY_SCALE = 0
LEGS_FITNESS_SCALE = 0
CONTACT_SCALE = 0
