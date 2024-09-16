import numpy as np

from Destination import Destination
from FLS import FLS
from State import StateTypes
from util import *


class Controller:
    def __init__(self, flss, flight_pattern, time_step, delta, schedule=None, consume_step=1):
        self.flss = flss
        self.flight_pattern = flight_pattern
        self.time_step = time_step
        self.delta = delta
        self.schedule = schedule
        self.consume_step = consume_step
        self.step = 0

    def social_interaction(self, fls_i, fls_j, config):
        # dz_ij = fls_j.position[2] - fls_i.position[2]
        dc_ij = np.linalg.norm(fls_i.position - fls_j.position)
        perceive_angles = vector_to_angles(fls_j.position - fls_i.position)

        psi_ij_3D = perceive_angles - fls_i.heading

        phi_ij_3D = fls_j.heading - fls_i.heading

        delta_vij = 0
        delta_phiAli_ij_3D, delta_phiAtt_ij_3D = np.zeros(2), np.zeros(2)
        for i in range(len(psi_ij_3D)):
            psi_ij = psi_ij_3D[i]
            phi_ij = phi_ij_3D[i]

            # if fls_i.ID == 0:
            print(f"{psi_ij:.2f}, {dc_ij:.2f}")
            delta_vij += config.gamma_Acc * np.cos(psi_ij) * (dc_ij - config.d_v_0) / (
                    1 + dc_ij / config.l_Acc)
            # delta_vzij = config.gamma_z * (
            #             np.tanh(dz_ij / config.a_z) + np.tanh((dz_ij - config.d_z_0) / config.a_z)) * np.exp(
            #     -(dc_ij / config.L_z_2) ** 2)
            if fls_j.state == StateTypes.SYNC:
                delta_phiAli_ij_3D[i] = 0
            else:
                delta_phiAli_ij_3D[i] = config.gamma_Ali * (dc_ij + config.d_Ali_0) * np.exp(
                    -(dc_ij / config.l_Ali) ** 2) * np.sin(phi_ij) * (1 + config.alpha_Ali * np.cos(psi_ij)) * (
                                                1 - self.f_w(fls_i, config.l_w, config.center, config.radius))
            delta_phiAtt_ij_3D[i] = (
                    config.gamma_Att * ((dc_ij - config.d_Att_0) / (1 + (dc_ij / config.l_Att) ** 2)) *
                    np.sin(psi_ij) * (1 - config.alpha_Att * np.cos(psi_ij)) *
                    (1 - self.f_w(fls_i, config.l_w, config.center, config.radius)) *
                    self.q(dc_ij, config))

        return delta_vij, delta_phiAli_ij_3D, delta_phiAtt_ij_3D

    def target_destination(self, fls_i, config):
        destination = self.flight_pattern.get_slot_coord(fls_i.slot_ID)

        d_Dest = np.linalg.norm(destination - fls_i.position)

        phi_dest = np.zeros(2)

        theta_d = vector_to_angles(destination - fls_i.position) - fls_i.heading

        for i in range(len(fls_i.heading)):
            phi_dest[i] = (config.gamma_Dest_h * (1 / (1 + (d_Dest / config.l_Dest) ** 2)) * np.sin(theta_d[i]) *
                           (1 - self.f_w(fls_i, config.l_w, config.center, config.radius)))

        v_dest = (config.gamma_Dest_v * (np.tanh(d_Dest / config.alfa_Dest_d) +
                                         np.tanh(config.v_Dest - fls_i.velocity / config.alfa_Dest_v)) *
                  np.exp(-(d_Dest / config.l_Dest) ** 2))

        print(f"Target_Delta_V: {v_dest:.2f}， FLS Speed: {fls_i.velocity:.2f}, Dist: {d_Dest:.2f}")

        return [v_dest, phi_dest]

    def closest_point_on_cuboid(self, position, vertices):

        mins = [np.min(vertices[:, 0]), np.min(vertices[:, 1]), np.min(vertices[:, 2])]
        maxs = [np.max(vertices[:, 0]), np.max(vertices[:, 1]), np.max(vertices[:, 2])]

        closest_point = np.zeros(3)
        for i in range(3):
            closest_point[i] = mins[i] if abs(position[i] - mins[i]) < abs(maxs[i] - position[i]) else maxs[i]

        return closest_point

    def get_r_w(self, fls, cylinder_center, radius):
        # IF Cuboid Shape
        # closest_point = self.closest_point_on_cuboid(fls.position, vertices)
        # r_w = np.linalg.norm(fls.position[:2] - closest_point[:2])

        d = np.linalg.norm(fls.position[:2] - cylinder_center)
        r_w = radius - d
        return r_w

    def get_theta_w(self, fls, cylinder_center):
        # IF Cuboid Shape
        # closest_point = self.closest_point_on_cuboid(fls.position, vertices)
        # wall_vec = np.array(closest_point - fls.position)
        #
        # wall_vec = wall_vec / np.linalg.norm(wall_vec)
        #
        # theta_w = np.arccos(np.dot(wall_vec[:2], [np.cos(fls.heading), np.sin(fls.heading)]))

        wall_vec = np.array(fls.position[:2] - cylinder_center)

        theta_c = get_heading(wall_vec[:2])

        theta_w = (theta_c - fls.heading[0])

        if theta_w > 2 * np.pi:
            theta_w -= 2 * np.pi
        elif theta_w < 0:
            theta_w += 2 * np.pi

        return theta_w

    def f_w(self, fls, l_w, cylinder_center, radius):
        r_w = self.get_r_w(fls, cylinder_center, radius)
        return np.exp(-(r_w / l_w) ** 2)

    def wall_effect(self, fls, config):
        theta_w = self.get_theta_w(fls, config.center)
        O_w = config.e_w1 * np.cos(theta_w) + config.e_w2 * np.cos(2 * theta_w)
        f_w = self.f_w(fls, config.l_w, config.center, config.radius)
        delta_phi_w = config.gamma_w * np.sin(theta_w) * (1 + O_w) * f_w

        # if fls.ID == 0:
        #     print(f"Theta: {theta_w * 180/np.pi:.2f}, r_w: {self.get_r_w(fls, config.center, config.radius)}, Position: {fls.position[:2]} ", end='')
        return -delta_phi_w

    def q(self, dc_ij, config):
        if dc_ij <= config.d_Att_0:
            return 2 * dc_ij / (4 * dc_ij - config.d_Att_0)
        return 1

    def compute_influence(self, delta_vij, delta_vzij, delta_phiij, velocity_i):
        return np.sqrt(delta_vij ** 2 + delta_vzij ** 2 + (delta_phiij * velocity_i) ** 2)

    def compute_influence_3D(self, delta_vij, delta_phiij_3D, velocity_i):
        return np.sqrt(delta_vij ** 2 + (delta_phiij_3D[0] * velocity_i) ** 2 + (delta_phiij_3D[1] * velocity_i) ** 2)

    def vertical_navigation_term(self, z, z_alt, gamma_perp, az):
        return -gamma_perp * np.tanh((z - z_alt) / az)

    def vertical_smoothing_term(self, vz, vi, gamma_parallel):
        if vi == 0:
            return 0
        else:
            return -gamma_parallel * vz / vi

    def update_FLS_swarm(self, config):
        if self.flight_pattern is not None:
            self.flight_pattern.update_slots()
        updateInfo = []
        for fls_i in self.flss:
            if fls_i.state == StateTypes.SYNC and self.flight_pattern is not None:
                heading_change = vector_to_angles(self.flight_pattern.get_slot_coord(fls_i.slot_ID, 1) -
                                                  self.flight_pattern.get_slot_coord(fls_i.slot_ID))
                delta_phi = heading_change - fls_i.heading
                updateInfo.append([0, delta_phi])
                continue

            influences = []
            deltas = []
            for fls_j in self.flss:

                if fls_i != fls_j:
                    delta_vij, delta_phiAli_ij_3D, delta_phiAtt_ij_3D = self.social_interaction(fls_i, fls_j, config)

                    influence = self.compute_influence_3D(delta_vij, delta_phiAli_ij_3D + delta_phiAtt_ij_3D,
                                                          fls_i.velocity)
                    influences.append(influence)
                    deltas.append((delta_vij, delta_phiAli_ij_3D, delta_phiAtt_ij_3D))

            delta_v, delta_vz, delta_phi = 0, 0, np.zeros(2)

            if influences:
                sorted_indices = np.argsort(influences)[::-1]

                adjacent_indices = sorted_indices[:2]

                for index in adjacent_indices:
                    delta_v_j, delta_phiAli_ij, delta_phiAtt_ij = deltas[index]
                    delta_v += delta_v_j
                    delta_phi += delta_phiAli_ij + delta_phiAtt_ij
                    # if fls_i.ID == 0:
                    #     print("Soc_Ali: " + f"{delta_phiAli_ij * 180/np.pi}",
                    #           "Soc_Att: " + f"{delta_phiAtt_ij * 180/np.pi}", end='')

            delta_phi_w = self.wall_effect(fls_i, config)

            # if fls_i.ID == 0:
            #     print("Wall_Delta_Phi: " + f"{np.degrees(delta_phi_w):.2f}")
            delta_phi[0] += delta_phi_w

            if fls_i.slot_ID >= 0:
                [delta_v_d, delta_phi_d] = self.target_destination(fls_i, config)
                delta_v += delta_v_d
                delta_phi += delta_phi_d

            updateInfo.append([delta_v, delta_phi])

        for i, fls_i in enumerate(self.flss):
            fls_i.update_state(updateInfo[i][0], updateInfo[i][1])

            if self.flight_pattern is not None:
                fls_i.sync_check(self.flight_pattern.get_slot_coord(fls_i.slot_ID))

    def predict_slots(self, fls, max_search_step, policy=0):
        # destination, expiration = self.shortest_time_match(fls, max_search_step)

        if policy == 0:  # FRT
            destination, expiration = self.shortest_time_match_bisearch(fls, max_search_step)
        else:  # SD
            destination, expiration = self.shortest_dist_match(fls, max_search_step)
            fls.destination.expected_arrive_time = ceil(expiration * self.time_step, 8)

        fls.destination.coordinate = destination
        fls.destination.expiration = int(expiration)

    def update_FLSs_linear(self, step_end_time):

        end_flag = True
        for fls in self.flss:
            travel_time = self.time_step
            if fls.state == StateTypes.STATIC:
                if (self.schedule[fls.ID])['departureT'] < step_end_time:

                    travel_time = step_end_time - (self.schedule[fls.ID])['departureT']

                    fls.assign_destination((self.schedule[fls.ID])['coord'], (self.schedule[fls.ID])['departureT'] - (self.schedule[fls.ID])['arrivalT'] - travel_time)
                    fls.assign_time = self.step * self.time_step
                    fls.depart_time = self.step * self.time_step
                else:
                    continue
            elif fls.state == StateTypes.QUIT:
                continue

            if distance(fls.destination.coordinate, fls.position) < (fls.max_speed**2 - fls.velocity**2)/(2 * fls.max_acc):
                max_speed = ((2 * fls.max_acc * distance(fls.destination.coordinate, fls.position)) - fls.velocity**2)
            else:
                max_speed = fls.max_speed

            arrived_flag = self.fls_goto_linear(fls.ID, fls.destination.coordinate, travel_time=travel_time,
                                                end_speed=max_speed)

            if arrived_flag:
                fls.state = StateTypes.QUIT
                # print(step_end_time - self.schedule[fls.ID]['arrivalT'])
                fls.destination = Destination()
                fls.path = []
                sync_flag = self.fls_sync_check(fls, self.delta)
                end_flag = end_flag & sync_flag

        return end_flag

    def update_FLSs_linear_fp(self, config, speed_error=0, heading_error=0, c=float('inf'), redeploy_flag=True):

        deltas = []
        end_flag = True
        exit_flag = self.flight_pattern.update_consume_step()

        for fls_i in self.flss:

            if fls_i.state == StateTypes.STATIC:
                assigned_slot_IDs = self.flight_pattern.assign_slot(1)
                if assigned_slot_IDs[0] >= 0:
                    fls_i.state = StateTypes.DYN
                    fls_i.slot_ID = assigned_slot_IDs[0]
                else:
                    continue

            if fls_i.state == StateTypes.DYN and fls_i.destination.expiration <= 0:
                self.predict_slots(fls_i, c, config.path_policy)
                fls_i.assign_time = self.step * self.time_step
                # if add_one_step:
                #     fls_i.destination.expiration += 1
                #     fls_i.destination.expected_arrive_time += fls_i.time_step

                if fls_i.destination.expected_arrive_time is not None:
                    delta = fls_i.destination.expected_arrive_time - np.linalg.norm(
                        fls_i.position - fls_i.destination.coordinate) / (config.v_Dest / 2)
                    if delta < 0:
                        delta = 0
                    deltas.append(f"          \u03B4: {delta:.2f} s")
            elif fls_i.destination.expected_arrive_time is not None:
                deltas.append(" ")

            if fls_i.state == StateTypes.END:

                if fls_i.slot_ID >= 0:
                    self.flight_pattern.free_slot(fls_i.slot_ID)
                    fls_i.slot_ID = -1

                if redeploy_flag:
                    dist = fls_i.velocity / 2 * fls_i.velocity / fls_i.max_acc
                    destination = np.array([fls_i.position[0], fls_i.position[1], fls_i.position[2] + dist])
                    arrived_flag = self.fls_goto_linear(fls_i.ID, destination)
                    if arrived_flag:
                        fls_i.state = StateTypes.CORNER

            elif fls_i.state == StateTypes.CORNER:
                destination = np.array([config.space, config.space, fls_i.position[2]])
                arrived_flag = self.fls_goto_linear(fls_i.ID, destination)
                if arrived_flag:
                    fls_i.state = StateTypes.MOVE_DOWN

            elif fls_i.state == StateTypes.MOVE_DOWN:
                destination = np.array([config.space, config.space, config.init_altitude])
                arrived_flag = self.constant_speed(fls_i.ID, destination, 0.5, fls_i.time_step)
                if arrived_flag:
                    fls_i.state = StateTypes.REDEPLOY
                    # points = generate_points(-config.space, config.space, -config.space, config.space,
                    #                          config.init_altitude, config.init_altitude, 1, 2 * config.fls_size)
                    points = get_collision_free_points(config.init_altitude)
                    fls_i.destination.coordinate = points[0]
                    fls_i.destination.expiration = float('inf')
                    fls_i.destination.expected_arrive_time = None

            elif fls_i.state == StateTypes.LAND:
                destination = np.array([fls_i.position[0], fls_i.position[1], 0.02])
                arrived_flag = self.constant_speed(fls_i.ID, destination, 0.5, fls_i.time_step)
                if arrived_flag:
                    fls_i.state = StateTypes.QUIT

            elif fls_i.state == StateTypes.QUIT:
                if fls_i.slot_ID >= 0:
                    self.flight_pattern.free_slot(fls_i.slot_ID)
                    fls_i.slot_ID = -1
                continue

            elif fls_i.state == StateTypes.REDEPLOY:

                arrived_flag = self.fls_goto_linear(fls_i.ID, fls_i.destination.coordinate)
                if arrived_flag:
                    assigned_slot_IDs = self.flight_pattern.assign_slot(1)
                    fls_i.redeploy(fls_i.position, assigned_slot_IDs[0])

            elif fls_i.state == StateTypes.SYNC:
                moving_vec = self.flight_pattern.get_slot_coord(fls_i.slot_ID, 1) - fls_i.position
                dist = np.linalg.norm(moving_vec)

                fls_i.update_state_linear(dist, config.v_Dest, moving_vec)

                if self.flight_pattern.get_time_to_coord(fls_i.slot_ID,
                                                         self.flight_pattern.depart_pos) < 2 * self.flight_pattern.time_step:
                    if exit_flag:
                        fls_i.state = StateTypes.EXIT
                        exit_flag = False
                        self.flight_pattern.exit_step = self.consume_step

                    # print(fls_i.position)
                continue

            elif fls_i.state == StateTypes.EXIT:
                moving_vec = (angles_to_vector(fls_i.heading) * fls_i.velocity * fls_i.time_step +
                              1 / 2 * fls_i.max_acc * self.flight_pattern.normal_vector * fls_i.time_step ** 2)

                dist = np.linalg.norm(moving_vec)
                moving_vec = moving_vec / dist

                velocity = np.linalg.norm(angles_to_vector(fls_i.heading) * fls_i.velocity +
                                          fls_i.max_acc * self.flight_pattern.normal_vector * fls_i.time_step)
                fls_i.update_state_linear(dist, velocity, moving_vec)

                if (np.dot(fls_i.position - self.flight_pattern.center, self.flight_pattern.normal_vector)
                        > self.flight_pattern.dist_to_opening):

                    if redeploy_flag:
                        fls_i.state = StateTypes.END
                    else:
                        fls_i.state = StateTypes.QUIT

            else:
                # FLS State is DYN
                last_speed = fls_i.velocity

                arrived_flag = self.fls_goto_linear(fls_i.ID, fls_i.destination.coordinate, end_speed=config.v_Dest)


                if last_speed == 0 and fls_i.velocity > 0:
                    fls_i.depart_time = self.step * self.time_step

                if arrived_flag:
                    fls_i.state = StateTypes.SYNC
                    fls_i.destination = Destination()
                    sync_flag = self.fls_sync_check(fls_i, self.delta)
                    end_flag = end_flag & sync_flag

        self.flight_pattern.update_slots()

        return end_flag, deltas

    def compute_dispersion(self):
        positions = self.get_positions()
        barycenter = np.mean(positions, axis=0)
        dispersion = np.sqrt(np.mean(np.linalg.norm(positions - barycenter, axis=1) ** 2))
        return dispersion

    def compute_polarization(self):
        velocities = np.array([fls.velocity for fls in self.flss])
        unit_velocities = velocities / np.linalg.norm(velocities, axis=0)
        polarization = np.linalg.norm(np.mean(unit_velocities, axis=0))
        return polarization

    def compute_milling_index(self):
        positions = self.get_positions()
        barycenter = np.mean(positions, axis=0)
        angles = np.arctan2(positions[:, 1] - barycenter[1], positions[:, 0] - barycenter[0])
        headings = np.array([fls.heading for fls in self.flss])
        milling_index = np.abs(np.mean(np.sin(headings - angles)))
        return milling_index

    def get_positions(self, check_states=False):
        if check_states:
            positions = []
            for fls in self.flss:
                if fls.state != StateTypes.QUIT:
                    positions.append(fls.position)
            return np.array(positions)
        else:
            return np.array([fls.position for fls in self.flss])

    def get_paths(self):
        return [fls.path for fls in self.flss]

    def get_slots(self):
        if not self.flight_pattern:
            return []
        return self.flight_pattern.slots

    def get_behavior(self):
        D = self.compute_dispersion()
        P = self.compute_polarization()
        M = self.compute_milling_index()

        return [D, P, M]

    def shortest_time_match(self, fls_i, max_search_step=float('inf')):

        sync_step = 0
        travel_time = float('inf')
        slot_position = np.zeros(3)

        while (sync_step * self.time_step) < travel_time and sync_step < max_search_step:
            slot_position = self.flight_pattern.get_slot_coord(fls_i.slot_ID, sync_step)

            dist = np.linalg.norm(slot_position - fls_i.position)

            distend_speed, dist_traveled, travel_time = fls_i.linear_movement_OPT(dist, self.flight_pattern.slot_speed,
                                                                                  float('inf'))

            sync_step += 1

        return slot_position, sync_step

    def shortest_time_match_bisearch(self, fls, max_search_step=float('inf')):

        closest_point, farthest_point = closest_farthest_point_on_circle(
            self.flight_pattern.normal_vector, self.flight_pattern.center, self.flight_pattern.radius, fls.position)

        dist_lower_bound = np.linalg.norm(closest_point - fls.position)
        dist_upper_bound = np.linalg.norm(farthest_point - fls.position)

        _, _, time_lower_bound = fls.linear_movement_OPT(dist_lower_bound, self.flight_pattern.slot_speed,
                                                         float('inf'))
        _, _, time_upper_bound = fls.linear_movement_OPT(dist_upper_bound, self.flight_pattern.slot_speed,
                                                         float('inf'))

        lo = min(floor(time_lower_bound / fls.time_step), max_search_step)
        hi = min(ceil(time_upper_bound / fls.time_step), max_search_step)
        theo_upper = hi

        if max_search_step <= lo:
            return self.flight_pattern.get_slot_coord(fls.slot_ID, steps=max_search_step + 1), max_search_step

        while lo < hi:
            mid = (lo + hi) // 2

            destination = self.flight_pattern.get_slot_coord(fls.slot_ID, steps=mid + 1)
            dist = np.linalg.norm(destination - fls.position)
            _, _, t = fls.linear_movement_OPT(dist, self.flight_pattern.slot_speed, float('inf'))

            time_diff = t - mid * fls.time_step

            if time_diff > 0:
                lo = mid + 1
            else:
                hi = mid

        # step_to_rendazvous = min(theo_upper, lo+1)
        return self.flight_pattern.get_slot_coord(fls.slot_ID, steps=lo), lo

    def shortest_dist_match(self, fls, max_search_step=float('inf')):

        closest_point, _ = closest_farthest_point_on_circle(
            self.flight_pattern.normal_vector, self.flight_pattern.center, self.flight_pattern.radius, fls.position)

        shortest_dist = np.linalg.norm(closest_point - fls.position)

        _, _, min_travel_time = fls.linear_movement_OPT(shortest_dist, self.flight_pattern.slot_speed,
                                                        float('inf'))

        slot_time = self.flight_pattern.get_time_to_coord(fls.slot_ID, closest_point)

        if slot_time < min_travel_time:
            round_time = 2 * np.pi / self.flight_pattern.rotation_speed
            slot_time += ceil((min_travel_time - slot_time) / round_time) * round_time

        return closest_point, min(ceil(slot_time / self.flight_pattern.time_step), max_search_step)

    def fls_sync_check(self, fls, threshold=0.1):
        sync_flag = False

        if not self.flight_pattern:
            return True

        if ((fls.state == StateTypes.SYNC or np.linalg.norm(fls.destination.coordinate - fls.position) < threshold)
                and self.flight_pattern.get_time_to_coord(fls.slot_ID,
                                                          fls.destination.coordinate) < self.flight_pattern.time_step):
            fls.set_sync_state()
            sync_flag = True

        return sync_flag

    def fls_goto_linear(self, fls_ID, destination, travel_time=None, end_speed=0, heading_error=0, speed_error=0):
        moving_vec = destination - self.flss[fls_ID].position
        dist = np.linalg.norm(moving_vec)

        if dist != 0:
            heading = moving_vec / dist

            factor_heading_error = [np.random.uniform(min([heading_error, 0]), max([heading_error, 0])) for _ in
                                    range(2)]
            heading = angles_to_vector(vector_to_angles(heading) + factor_heading_error)
        else:
            heading = np.array([0, 0, 0])

        end_speed, dist_traveled = self.flss[fls_ID].make_move(dist, end_speed, travel_time,
                                                               speed_error=speed_error / 2)

        self.flss[fls_ID].update_state_linear(dist_traveled, end_speed, heading)

        if np.linalg.norm(destination - self.flss[fls_ID].position) < 1e-6:
            return True

        return False

    def constant_speed(self, fls_ID, destination, speed, duration):
        moving_vec = destination - self.flss[fls_ID].position
        dist = np.linalg.norm(moving_vec)

        if dist != 0:
            heading = moving_vec / dist
            heading = angles_to_vector(vector_to_angles(heading))
        else:
            heading = np.array([0, 0, 0])

        dist_traveled = min(duration * speed, dist)

        self.flss[fls_ID].update_state_linear(dist_traveled, speed, heading)

        if np.linalg.norm(destination - self.flss[fls_ID].position) < 1e-6:
            return True

        return False
