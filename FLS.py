import math

import numpy as np

from Destination import Destination
from State import StateTypes
from util import *


class FLS:
    def __init__(self, ID, position, heading, velocity, time_step, speed_range, acc_range=[float('inf'), float('inf')],
                 slot_ID=-1):
        self.ID = ID
        self.position = np.array(position, dtype=float)
        self.heading = np.array(heading, dtype=float)
        self.velocity = velocity
        self.time_step = time_step
        self.max_speed = speed_range[0]
        self.min_speed = speed_range[1]
        self.max_acc = acc_range[0]
        self.max_dec = acc_range[1]
        self.path = [position]
        self.slot_ID = slot_ID
        self.depart_time = 0
        self.assign_time = 0

        if slot_ID >= 0:
            self.state = StateTypes.DYN
        else:
            self.state = StateTypes.STATIC
        self.destination = Destination()
        self.dist_traveled = 0

    def __str__(self):
        return f"{self.ID} {self.state.value} {self.position} {self.heading} {self.velocity}"

    def update_state_linear(self, dist, v, heading_vec):
        self.velocity = v
        if np.linalg.norm(heading_vec) == 0:
            heading_vec = np.array([0.0, 0.0, 0.0])
        else:
            heading_vec = heading_vec / np.linalg.norm(heading_vec)

        self.heading = vector_to_angles(heading_vec)

        for i in range(len(self.heading)):
            if self.heading[i] > 2 * np.pi:
                self.heading[i] -= 2 * np.pi
            elif self.heading[i] < 0:
                self.heading[i] += 2 * np.pi

        self.position += dist * heading_vec

        self.dist_traveled += dist

        self.add_path()

    def update_state(self, delta_v, delta_phi):
        v_0 = self.velocity
        self.velocity += delta_v
        if self.velocity < self.min_speed:
            self.velocity = self.min_speed
        elif self.velocity > self.max_speed:
            self.velocity = self.max_speed

        for i in range(len(delta_phi)):
            self.heading[i] += min([delta_phi[i], (np.pi / 6) * self.time_step])
            # self.heading[i] += delta_phi[i]

            if self.heading[i] > 2 * np.pi:
                self.heading[i] -= 2 * np.pi
            elif self.heading[i] < 0:
                self.heading[i] += 2 * np.pi

        dist = 0.5 * (v_0 + self.velocity) * self.time_step
        self.position += dist * angles_to_vector(self.heading)

        self.dist_traveled += dist

        # if self.ID ==0:
        #     print(f"{get_heading([np.cos(self.heading), np.sin(self.heading)], [1, 0]) * 180/np.pi:.4f}")
        #     print(f"{get_heading(self.position[:2] - self.path[-1][:2], [1, 0]) * 180/np.pi:.4f}, ",
        #           f"velocity: {self.velocity}")

        self.add_path()

    def add_path(self):
        self.path.append(self.position.copy())
        if len(self.path) > 20:
            self.path.pop(0)

    def get_moving_limitation(self, cur_speed, dist, arrival_speed, t):
        max_acc_dist = (self.max_speed ** 2 - cur_speed ** 2) / (2 * self.max_acc)
        max_dec_dist = (self.max_speed ** 2 - arrival_speed ** 2) / (2 * self.max_dec)
        min_dec_dist = (cur_speed ** 2 - arrival_speed ** 2) / (2 * self.max_dec)

        if (dist > max_acc_dist + max_dec_dist + self.max_speed * t) or (
                t == float('inf') and dist > max_acc_dist + max_dec_dist):
            max_speed_dist = dist - max_acc_dist - max_dec_dist
            t_max = max_speed_dist / self.max_speed
            t_acc = (self.max_speed - cur_speed) / self.max_acc
            t_dec = (self.max_speed - arrival_speed) / self.max_dec
            top_speed = self.max_speed
        elif dist > min_dec_dist:
            a = 1 / (2 * self.max_acc) + 1 / (2 * self.max_dec)
            c = -cur_speed ** 2 / (2 * self.max_acc) - arrival_speed ** 2 / (2 * self.max_dec) - dist

            top_speed = math.sqrt(-4 * a * c) / (2 * a)
            t_acc = (top_speed - cur_speed) / self.max_acc
            t_dec = (top_speed - arrival_speed) / self.max_dec
            t_max = 0
        else:
            t_acc = 0
            t_max = 0
            t_dec = (cur_speed - arrival_speed) / self.max_dec
            top_speed = cur_speed

        return t_acc, t_max, t_dec, top_speed

    def linear_movement_OPT(self, dist, arrival_speed, t, error=0.0):

        t_acc, t_max, t_dec, top_speed = self.get_moving_limitation(self.velocity, dist, arrival_speed, t)

        dist_traveled = 0
        end_speed = 0

        factor = 1 + np.random.uniform(min([2 * error, 0]), max([2 * error, 0]))

        top_speed = top_speed * factor

        if t <= t_acc:
            end_speed = self.velocity + t * self.max_acc * factor

            dist_traveled += t * (self.velocity + end_speed) / 2
        elif t_max > 0 and t <= t_acc + t_max:
            t_actual_max_speed = t - t_acc
            top_speed_after_error = self.velocity + (top_speed - self.velocity) * factor

            dist_traveled = t_acc * (
                    self.velocity + top_speed_after_error) / 2 + t_actual_max_speed * top_speed_after_error
            end_speed = top_speed_after_error

        elif t > t_acc + t_max:
            t_actual_dec = min(t - t_acc - t_max, t_dec)
            t = t_acc + t_max + t_actual_dec

            top_speed_after_error = self.velocity + (top_speed - self.velocity) * factor
            end_speed = top_speed_after_error - (t_actual_dec * self.max_dec * factor)

            dist_traveled = (
                    t_acc * (self.velocity + top_speed_after_error) / 2
                    + t_max * top_speed_after_error
                    + t_actual_dec * (top_speed_after_error + end_speed) / 2
            )

        return end_speed, dist_traveled, t


    def linear_movement_specified_time(self, dist, arrival_speed, t, error=0.0):
        total_time = self.destination.expiration * self.time_step

        avg_speed = dist / total_time

        if avg_speed > self.max_speed or avg_speed < self.min_speed:
            print(f"ERROR: Avg speed: {avg_speed}, Current Speed: {self.velocity}")

        opt_avg_speed = (self.velocity + arrival_speed) / 2

        waiting_dist = 0

        # First accelerate later decelerate. If no acceleration process, should go to the last 'else' condition
        if dist - opt_avg_speed * total_time > 1e-8:

            # Stay stationary as long as possible before moving
            if self.velocity == 0:
                max_acc_time = (self.max_speed - self.velocity) / self.max_acc
                max_dec_time = (self.max_speed - arrival_speed) / self.max_dec

                if max_dec_time + max_acc_time <= total_time:
                    max_speed = self.max_speed

                else:
                    max_acc_time = (arrival_speed + self.max_dec * total_time - self.velocity) / (
                                self.max_acc + self.max_dec)
                    max_dec_time = total_time - max_acc_time
                    max_speed = max_acc_time * self.max_acc + self.velocity

                dist_upperbound = (max_acc_time * (self.velocity + max_speed) / 2 +
                                   max_dec_time * (arrival_speed + max_speed) / 2 +
                                   max_speed * (total_time - max_acc_time - max_dec_time)
                                   )

                if dist < dist_upperbound:
                    waiting_time = (dist_upperbound - dist) / (max_speed - self.velocity)
                    if waiting_time > t:
                        return self.velocity, self.velocity * t
                    else:
                        t = t - waiting_time
                        total_time = total_time - waiting_time
                        waiting_dist = waiting_time * self.velocity

            if (arrival_speed - self.velocity) / (2 * self.max_dec) + total_time / 2 == 0:
                top_speed = arrival_speed
            else:
                top_speed = (dist + arrival_speed * (arrival_speed - self.velocity) / (
                        2 * self.max_dec) - self.velocity * total_time / 2) / (
                                    (arrival_speed - self.velocity) / (2 * self.max_dec) + total_time / 2)

            t_dec = (top_speed - arrival_speed) / self.max_dec
            t_cruise = 0
            t_acc = total_time - t_dec - t_cruise

            if t_dec * self.max_dec + arrival_speed > self.max_speed:
                top_speed = self.max_speed
                t_dec = (top_speed - arrival_speed) / self.max_dec
                # t_cruise = ((top_speed + arrival_speed) * t_dec + (total_time - t_dec) * (
                #             top_speed + self.velocity) / 2) / ((top_speed - self.velocity) / 2)

                if self.velocity == top_speed:
                    t_cruise = (dist - (top_speed + arrival_speed) * t_dec) / top_speed
                    t_acc = total_time - t_dec - t_cruise
                else:
                    t_acc = (dist - t_dec * (top_speed + arrival_speed)/2 - (total_time - t_dec) * top_speed) / ((self.velocity - top_speed) / 2)
                    t_cruise = total_time - t_dec - t_acc

            if t <= t_acc:
                if t_acc == 0:
                    end_speed = self.velocity
                else:
                    end_speed = t * (top_speed - self.velocity) / t_acc + self.velocity
                dist_traveled = t * (self.velocity + end_speed) / 2

            elif t <= t_acc + t_cruise:
                end_speed = top_speed
                dist_traveled = t_acc * (self.velocity + top_speed) / 2 + (t - t_acc) * top_speed

            else:  # FLS finished acceleration and have began decelerating
                end_speed = top_speed - (t - t_acc - t_cruise) * (top_speed - arrival_speed) / (
                        total_time - t_acc - t_cruise)
                dist_traveled = t_acc * (self.velocity + top_speed) / 2 + t_cruise * top_speed + (
                        t - t_acc - t_cruise) * (top_speed + end_speed) / 2

        # First decelerate later accelerate. If no acceleration process (current speed == 0), should go to the last 'else' condition
        elif dist - opt_avg_speed * total_time < -1e-8 and self.velocity > 0:

            print('Error Moving State')

            if (self.velocity - arrival_speed) / (2 * self.max_acc) + total_time / 2 == 0:
                min_speed = arrival_speed
            else:
                min_speed = (
                            (dist - total_time * self.velocity / 2 + (arrival_speed * (self.velocity - arrival_speed)) /
                             (2 * self.max_acc)) / (
                                    (self.velocity - arrival_speed) / (2 * self.max_acc) + total_time / 2))

            t_acc = (arrival_speed - min_speed) / self.max_acc
            t_cruise = 0
            t_dec = total_time - t_acc - t_cruise

            if arrival_speed - t_acc * self.max_acc < 0:
                min_speed = 0
                t_acc = (arrival_speed - min_speed) / self.max_acc
                t_cruise = (dist - t_acc * (min_speed + self.velocity) / 2 - (total_time - t_acc) * (
                            min_speed + arrival_speed) / 2) / ((min_speed - arrival_speed) / 2)

                t_dec = total_time - t_acc - t_cruise

            if t < t_dec:
                if t_dec == 0:
                    end_speed = self.velocity
                else:
                    end_speed = self.velocity - t * (self.velocity - min_speed) / t_dec
                dist_traveled = t * (self.velocity + end_speed) / 2

            elif t <= t_dec + t_cruise:
                end_speed = min_speed
                dist_traveled = t_dec * (self.velocity + min_speed) / 2 + (t - t_dec) * min_speed

            else:
                end_speed = min_speed + (t - t_dec - t_cruise) * (arrival_speed - min_speed) / (
                        total_time - t_dec - t_cruise)
                dist_traveled = t_dec * (self.velocity + min_speed) / 2 + t_cruise * min_speed + (
                        t - t_dec - t_cruise) * (min_speed + end_speed) / 2

        # Just accelerate later decelerate. Happen when speed = 0. Should wait
        elif dist - opt_avg_speed * total_time < -1e-8 and self.velocity == 0:
            min_acc_time = (arrival_speed - self.velocity)/self.max_acc
            dist_lowerbound = min_acc_time * (self.velocity + arrival_speed)/2
            waiting_time = (dist - dist_lowerbound)/((self.velocity + arrival_speed)/2)
            if t < waiting_time:
                return self.velocity, self.velocity * t
            else:
                end_speed = self.velocity + (t - waiting_time) * (arrival_speed - self.velocity)/(total_time - waiting_time)
                dist_traveled = self.velocity * waiting_time + (t - waiting_time) * (self.velocity + end_speed)/2
                return end_speed, dist_traveled

        else:
            end_speed = t * (arrival_speed - self.velocity) / total_time + self.velocity
            dist_traveled = t * (self.velocity + end_speed) / 2

        return end_speed, dist_traveled + waiting_dist

    def make_move(self, dist, arrival_speed, travel_time=None, speed_error=0.0):
        # end_speed, dist_traveled = self.linear_movement_specified_time(dist, arrival_speed, self.destination.expiration * self.time_step,
        #                                                                speed_error / 2)
        if not travel_time:
            travel_time = self.time_step

        if self.destination.expected_arrive_time is not None:
            end_speed, dist_traveled = self.linear_movement_specified_time(dist, arrival_speed, travel_time,
                                                                           speed_error / 2)
        else:
            end_speed, dist_traveled, _ = self.linear_movement_OPT(dist, arrival_speed, travel_time,
                                                                   speed_error / 2)

        self.destination.expiration -= 1

        if self.destination.expiration <= 0:
            self.destination.expiration += 1

        return end_speed, dist_traveled

    def set_sync_state(self):
        self.state = StateTypes.SYNC
        return

    def redeploy(self, position, slot_ID):
        self.position = position
        self.heading = [np.random.uniform(0, 2 * np.pi), np.random.uniform(0, 2 * np.pi)]
        self.velocity = 0
        self.path = [self.position]
        self.slot_ID = slot_ID
        self.state = StateTypes.DYN
        self.destination = Destination()

    def land(self):
        self.state = StateTypes.LAND

    def assign_destination(self, coord, time_to_arrive):
        self.destination = Destination(coordinate=coord, expiration=time_to_arrive/self.time_step)
        self.state = StateTypes.DYN