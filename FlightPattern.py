import numpy as np

from util import *


class FlightPattern:
    def __init__(self, center, radius, dist_to_opening, speed, slot_num, time_step, normal_vector=np.array([0, 0, 1]),
                 exit_step=0):
        normal_vector = normal_vector / np.linalg.norm(normal_vector)
        self.center = np.array(center)
        self.normal_vector = np.array(normal_vector)
        self.slots = generate_points_on_circle(center, radius, slot_num, normal=normal_vector)
        self.assignment = [False for _ in range(slot_num)]
        self.dist_to_opening = dist_to_opening
        self.radius = radius
        self.slot_speed = speed
        self.rotation_speed = speed / radius
        self.slot_num = np.array(slot_num)
        self.time_step = time_step
        self.rotation_matrix = self.get_rotation_matrix(self.rotation_speed * self.time_step)
        self.depart_pos = np.copy(self.slots[0]) if slot_num > 0 else None
        self.exit_step = exit_step

    def update_consume_step(self):
        self.exit_step -= 1

        if self.exit_step < 0:
            self.exit_step = 0

        if self.exit_step == 0:
            return True
        else:
            return False

    def get_rotation_matrix(self, angle):
        n = self.normal_vector
        angle_cos = np.cos(angle)
        angle_sin = np.sin(angle)

        rotation_matrix = np.array([
            [angle_cos + n[0] ** 2 * (1 - angle_cos), n[0] * n[1] * (1 - angle_cos) - n[2] * angle_sin,
             n[0] * n[2] * (1 - angle_cos) + n[1] * angle_sin],
            [n[1] * n[0] * (1 - angle_cos) + n[2] * angle_sin, angle_cos + n[1] ** 2 * (1 - angle_cos),
             n[1] * n[2] * (1 - angle_cos) - n[0] * angle_sin],
            [n[2] * n[0] * (1 - angle_cos) - n[1] * angle_sin, n[2] * n[1] * (1 - angle_cos) + n[0] * angle_sin,
             angle_cos + n[2] ** 2 * (1 - angle_cos)]
        ])

        return rotation_matrix

    def update_slots(self):
        if self.slot_num <= 0:
            return
        for i in range(self.slot_num):
            central_vec = self.slots[i] - self.center

            # Rotate the vector
            new_vec = np.dot(self.rotation_matrix, central_vec)

            # Calculate the new point's coordinates
            self.slots[i] = self.center + new_vec

    def get_slot_coord(self, slot_ID, steps=0):
        if slot_ID < 0:
            return None

        central_vec = self.slots[slot_ID] - self.center

        rotation_matrix = self.get_rotation_matrix(steps * self.rotation_speed * self.time_step)

        try:
            new_vec = np.dot(rotation_matrix, central_vec)
        except:
            print('Error')
            print(rotation_matrix, central_vec)
        central_vec = new_vec

        # Calculate the new point's coordinates
        return self.center + central_vec

    def get_time_to_coord(self, slot_ID, destination):
        if slot_ID < 0:
            return None

        radial_diff = get_counter_clockwise_angle(self.slots[slot_ID] - self.center, destination - self.center,
                                                  self.normal_vector)
        if radial_diff < 0:
            radial_diff += 2 * np.pi

        time = radial_diff / self.rotation_speed
        return ceil(time, 8)

    def assign_slot(self, assignment_num=1, assignment=None):
        if assignment is None:
            assign_list = []
            for i, assigned in enumerate(self.assignment):
                if not assigned:
                    self.assignment[i] = True
                    assign_list.append(i)
                    if len(assign_list) >= assignment_num:
                        break

                    # Handling slot num < FLS num is not handled
        else:
            assign_list = assignment
            for i in range(min(len(assign_list), assignment_num)):
                if not self.assignment[assign_list[i]]:
                    self.assignment[assign_list[i]] = True
                else:
                    assign_list[i] = -1

        if len(assign_list) < assignment_num:
            assign_list.extend([-1 for _ in range(assignment_num - len(assign_list))])

        return assign_list

    def free_slot(self, slot_ID):
        if slot_ID < 0:
            return None

        self.assignment[slot_ID] = False
        return
