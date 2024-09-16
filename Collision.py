from enum import Enum


class CollisionTypes(Enum):
    Static_Static = 0
    Moving_Static = 1
    Moving_Moving = 2
    Slot_Slot = 3
    Moving_Slot = 4
    Static_Slot = 5
    Exit_Exit = 6
    Slot_Exit = 7
    Moving_Exit = 8
    Static_Exit = 9
    Other = 10

