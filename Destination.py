import numpy as np


class Destination:

    def __init__(self, coordinate=np.zeros(3), expiration=0, expected_arrive_time=None):
        self.coordinate = coordinate
        self.expiration = expiration
        self.expected_arrive_time = expected_arrive_time
