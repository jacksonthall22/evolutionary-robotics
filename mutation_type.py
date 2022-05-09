from enum import Enum, auto


class MutationType(Enum):
    CONSTANT = auto()
    LINEAR = auto()
    DECAY = auto()
    NEGATIVE_EXPONENTIAL = auto()
