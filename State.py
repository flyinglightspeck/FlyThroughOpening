from enum import Enum


class StateTypes(Enum):
    STATIC = 0
    DYN = 1
    SYNC = 2
    EXIT = 3
    END = 4
    CORNER = 5
    MOVE_DOWN = 6
    REDEPLOY = 7
    LAND = 8
    QUIT = 9 # default end state: ignore all collisions with state value larger than given state
    DYN_STAION = 10

