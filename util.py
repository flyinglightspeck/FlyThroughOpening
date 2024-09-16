import collections
import csv
import math
import os
import statistics
import time
from collections import defaultdict

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import warnings
import re

from matplotlib.ticker import FuncFormatter
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from scipy.spatial.distance import pdist, squareform

from Collision import CollisionTypes
from State import StateTypes

count = 0


def get_heading(v1):
    v1 = v1 / np.linalg.norm(v1)
    v2 = [1, 0]

    included_angle = np.arccos(np.dot(v1, v2))

    if v1[1] < 0:
        included_angle = 2 * np.pi - included_angle

    return included_angle


def draw_ring(ax, center, inner_radius, width, color, alpha=0.6):
    # Angle values
    theta = np.linspace(0, 2 * np.pi, 100)

    outer_radius = inner_radius + width

    # Outer ring
    outer_x = center[0] + outer_radius * np.cos(theta)
    outer_y = center[1] + outer_radius * np.sin(theta)
    outer_z = np.full_like(theta, center[2])

    # Inner ring
    inner_x = center[0] + inner_radius * np.cos(theta)
    inner_y = center[1] + inner_radius * np.sin(theta)
    inner_z = np.full_like(theta, center[2])

    for i in range(len(outer_x) - 1):
        # Define the vertices of the face
        face_vertices = [
            [outer_x[i], outer_y[i], outer_z[i]],
            [outer_x[i + 1], outer_y[i + 1], outer_z[i + 1]],
            [inner_x[i + 1], inner_y[i + 1], inner_z[i + 1]],
            [inner_x[i], inner_y[i], inner_z[i]]
        ]
        # Create the face
        face = Poly3DCollection([face_vertices], color=color, alpha=alpha, linewidths=0, edgecolors=None)
        # Add the face to the plot
        ax.add_collection3d(face)

    return


def draw_sphere(ax, center, radius, color, alpha=0.6):
    # Create a sphere
    u = np.linspace(0, 2 * np.pi, 100)
    v = np.linspace(0, np.pi, 100)
    x = center[0] + radius * np.outer(np.cos(u), np.sin(v))
    y = center[1] + radius * np.outer(np.sin(u), np.sin(v))
    z = center[2] + radius * np.outer(np.ones(np.size(u)), np.cos(v))

    # Plot the sphere
    ax.plot_surface(x, y, z, color=color, alpha=alpha)


def draw_opening(ax, center, board_width, color, alpha=0.6, zorder=0):
    edge_length = 0.6

    # Calculate half the edge length
    half_edge = edge_length / 2 + (board_width - 0.03) / 2

    # Define the vertices of the outer square
    outer_vertices = np.array([
        [center[0] - half_edge, center[1] - half_edge, center[2]],
        [center[0] + half_edge, center[1] - half_edge, center[2]],
        [center[0] + half_edge, center[1] + half_edge, center[2]],
        [center[0] - half_edge, center[1] + half_edge, center[2]]
    ])

    # Define the vertices of the inner square (hole)
    inner_half_edge = half_edge - 0.03 - (board_width - 0.03) / 2
    inner_vertices = np.array([
        [center[0] - inner_half_edge, center[1] - inner_half_edge, center[2]],
        [center[0] + inner_half_edge, center[1] - inner_half_edge, center[2]],
        [center[0] + inner_half_edge, center[1] + inner_half_edge, center[2]],
        [center[0] - inner_half_edge, center[1] + inner_half_edge, center[2]]
    ])

    # Create the faces connecting the outer and inner squares
    for i in range(4):
        # Define the vertices of the face
        face_vertices = [
            outer_vertices[i],
            outer_vertices[(i + 1) % 4],
            inner_vertices[(i + 1) % 4],
            inner_vertices[i]
        ]
        # Create the face
        face = Poly3DCollection([face_vertices], color=color, alpha=alpha, linewidths=0, edgecolors=None, zorder=zorder)
        # Add the face to the plot
        ax.add_collection3d(face)


def draw_cylinder(ax, center, radius, height, color='r', alpha=0.2):
    x_center, y_center = center

    # Create the cylinder's base circle
    theta = np.linspace(0, 2 * np.pi, 50)
    x = radius * np.cos(theta) + x_center
    y = radius * np.sin(theta) + y_center

    # Create the top and bottom surfaces of the cylinder
    z_top = np.full_like(x, height)
    z_bottom = np.full_like(x, 0)

    # Plot the top and bottom surfaces
    ax.plot(x, y, z_bottom, color=color, alpha=alpha)
    ax.plot(x, y, z_top, color=color, alpha=alpha)

    # Create the cylindrical surface
    for i in range(len(theta)):
        ax.plot([x[i], x[i]], [y[i], y[i]], [0, height], color=color, alpha=alpha)


def generate_points(x_min, x_max, y_min, y_max, z_min, z_max, n, min_distance):
    points = []

    while len(points) < n:
        x = np.random.uniform(x_min, x_max)
        y = np.random.uniform(y_min, y_max)
        z = np.random.uniform(z_min, z_max)
        point = (x, y, z)
        if is_valid_point(point, points, min_distance):
            points.append(point)

    return np.array(points)


def get_collision_free_points(z):
    global count

    if count >= 10:
        count = 0
        print("Reset")

    points = [
        [-1.0319440786726903, -1.0320164389913922, z],
        [-0.7257491634954017, 1.0985284373248057, z],
        [-0.2041649440736526, -0.6263125794058743, z],
        [0.997327922401265, -0.8629826679651715, z],
        [0.3033450352296265, 0.6242177333881367, z],
        [-1.4382465171125927, 1.409729556485983, z],
        [0.3355586841671383, -1.0815184180438746, z],
        [-0.5872732711213868, 0.07426929489671341, z],
        [-0.3763796434579125, 1.3521429192297485, z],
        [0.6959818254342154, 0.2959754525911098, z]
    ]

    count += 1
    return np.array([points[count - 1]])


def is_valid_point(point, points, min_distance):
    for p in points:
        if np.linalg.norm(np.array(point) - np.array(p)) < min_distance:
            return False
    return True


def get_collision_num(points, min_distance):
    collision = 0
    for i, p in enumerate(points):
        for point in points[i + 1:]:
            if np.linalg.norm(np.array(point) - np.array(p)) < min_distance:
                collision += 1
    return collision


def check_collision_type(flss, min_distance):
    collisions = []
    for i, f in enumerate(flss):
        if f.state == StateTypes.QUIT:
            continue

        for fls in flss[i + 1:]:
            # check if collided

            if fls.state == StateTypes.QUIT:
                continue

            if np.linalg.norm(np.array(fls.position) - np.array(f.position)) < min_distance:
                collision_type = categorize_collisions(f.state, fls.state)

                collisions.append(collision_type)

    return collisions


def categorize_collisions(state_i, state_j):
    if state_j == StateTypes.QUIT or state_j == StateTypes.QUIT:
        print("FLS State Error")
        return CollisionTypes.Other

    if state_i == StateTypes.STATIC:
        if state_j == StateTypes.STATIC:
            return CollisionTypes.Static_Static
        elif state_j == StateTypes.DYN:
            return CollisionTypes.Moving_Static
        elif state_j == StateTypes.SYNC:
            return CollisionTypes.Static_Slot
        elif state_j == StateTypes.EXIT:
            return CollisionTypes.Static_Exit
        else:
            return CollisionTypes.Other


    elif state_i == StateTypes.DYN:
        if state_j == StateTypes.STATIC:
            return CollisionTypes.Moving_Static
        elif state_j == StateTypes.DYN:
            return CollisionTypes.Moving_Moving
        elif state_j == StateTypes.SYNC:
            return CollisionTypes.Moving_Slot
        elif state_j == StateTypes.EXIT:
            return CollisionTypes.Moving_Exit
        else:
            return CollisionTypes.Other


    elif state_i == StateTypes.SYNC:
        if state_j == StateTypes.STATIC:
            return CollisionTypes.Static_Slot
        elif state_j == StateTypes.DYN:
            return CollisionTypes.Moving_Slot
        elif state_j == StateTypes.SYNC:
            return CollisionTypes.Slot_Slot
        elif state_j == StateTypes.EXIT:
            return CollisionTypes.Slot_Exit
        else:
            return CollisionTypes.Other

    elif state_i == StateTypes.EXIT:
        if state_j == StateTypes.STATIC:
            return CollisionTypes.Static_Exit
        elif state_j == StateTypes.DYN:
            return CollisionTypes.Moving_Exit
        elif state_j == StateTypes.SYNC:
            return CollisionTypes.Slot_Exit
        elif state_j == StateTypes.EXIT:
            return CollisionTypes.Exit_Exit
        else:
            return CollisionTypes.Other
    else:
        return CollisionTypes.Other


def vector_to_angles(vector):
    if not any(vector):
        return np.array([0, 0])

    # Ensure the vector is normalized
    vector = vector / np.linalg.norm(vector)

    x, y, z = vector

    theta = np.arctan2(y, x)

    omega = np.arctan2(z, np.sqrt(x ** 2 + y ** 2))

    angles = np.array([theta, omega])

    for i, angle in enumerate(angles):
        if angle > 2 * np.pi:
            angle -= 2 * np.pi
        elif angle < 0:
            angle += 2 * np.pi
        angles[i] = angle

    return angles


def angles_to_vector(angles):
    theta, omega = angles

    z = np.sin(omega)

    xy_plane_length = np.sqrt(1 - z ** 2)
    x = np.cos(theta) * xy_plane_length
    y = np.sin(theta) * xy_plane_length

    vec = np.array([x, y, z])
    vec = vec / np.linalg.norm(vec)

    return vec


# def generate_points_on_circle(center, radius, n):
#     theta = np.linspace(0, 2 * np.pi, n+1)
#
#     points = []
#
#     for t in theta[:-1]:
#         x = center[0] + radius * np.cos(t)
#         y = center[1] + radius * np.sin(t)
#         z = center[2]
#         points.append((x, y, z))
#
#     return np.array(points)


def generate_points_on_circle(center, radius, n, normal=np.array([0, 0, 1])):
    normal = normal / np.linalg.norm(normal)  # Normalize the normal vector
    # Find an orthogonal vector
    if normal[2] != 0:
        v1 = np.array([1, 0, -normal[0] / normal[2]])
    else:
        v1 = np.array([0, 1, -normal[1] / normal[2]])
    v1 = v1 / np.linalg.norm(v1)
    # Find a second orthogonal vector using cross product
    v2 = cross_product(normal, v1)
    v2 = v2 / np.linalg.norm(v2)

    # Generate points on the circle
    points = []
    for i in range(n):
        t = 2 * np.pi * i / n
        point = center + radius * np.cos(t) * v1 + radius * np.sin(t) * v2
        points.append(point)

    return np.array(points)


def draw_line_chart(y_lists, x, line_names, x_label, y_label, save_name=None):
    markers = ['o', '^', 's', 'D', 'h', 'v']
    colors = ["#0072BD", "#7E2F8E", "#8B6C5C", "#edb120", "#000000", "#4DBEEE"]

    fig = plt.figure(figsize=(10, 6))
    ax = fig.add_subplot(111)
    for y, line_name, color, marker in zip(y_lists, line_names, colors, markers):
        ax.plot(x, y, marker=marker, linestyle='-', color=color, label=line_name)

        ax.text((x[3] + x[4]) / 2, y[3] + (max(max((y_lists))) - min(min(y_lists))) / 15, line_name, fontsize=28,
                ha='center', va='center', color=color, font='Times New Roman')

    # Add title and labels
    ax.set_xlabel(x_label, fontsize=30, font='Times New Roman')
    title_font = {'fontname': 'Times New Roman', 'fontsize': 30}
    ax.set_title(y_label, fontdict=title_font, loc='left', pad=0)
    # ax.set_ylabel(y_label, fontsize=16, font='Times New Roman')

    # Add a legend
    # plt.legend()

    ax.set_ylim(4, 16)
    ax.spines[['right', 'top']].set_visible(False)
    # Show the plot
    ax.grid(True)
    for label in (ax.get_xticklabels() + ax.get_yticklabels()):
        label.set_fontname('Times New Roman')
        label.set_fontsize(28)

    plt.tight_layout()
    if save_name is not None:
        fig.savefig(save_name, dpi=300)
    fig.show()


def draw_line_chart_percentage(y_lists, x, line_names, x_label, y_label, save_name=None):
    markers = ['o', 's', '^', 'D', 'h', 'v']
    colors = ["#0072BD", "#8B6C5C", "#7E2F8E", "#edb120", "#000000", "#4DBEEE"]

    fig = plt.figure(figsize=(10, 6))
    ax = fig.add_subplot(111)
    for i, data in enumerate(zip(y_lists, line_names, colors, markers)):
        y, line_name, color, marker = data
        ax.plot(x, y, marker=marker, linestyle='-', color=color, label=line_name)
        if i == 4:
            ax.text((x[2] + x[1]) / 2, y[2] - (max(max((y_lists))) - min(min(y_lists))) / 15, line_name, fontsize=20,
                    ha='center',
                    va='center', color=color, font='Times New Roman')
        else:
            ax.text((x[1] + x[1]) / 2, y[2] + (max(max((y_lists))) - min(min(y_lists))) / 20, line_name, fontsize=20,
                    ha='center', va='center', color=color, font='Times New Roman')

    # Add title and labels
    ax.set_xlabel(x_label, fontsize=20, font='Times New Roman')
    title_font = {'fontname': 'Times New Roman', 'fontsize': 18}
    # ax.set_title(y_label, fontdict=title_font, loc='left', pad=0)
    ax.text(0.2, 1.05, y_label, fontdict=title_font, fontsize=20)

    # ax.set_ylabel(y_label, fontsize=16, font='Times New Roman')

    def to_percent(y, position):
        # Convert y values to percentage
        s = f"{100 * y:.0f}"
        return s + '%'

    # Set the y-axis formatter to percentage
    ax.yaxis.set_major_formatter(FuncFormatter(to_percent))

    ax.spines[['right', 'top']].set_visible(False)

    for label in (ax.get_xticklabels() + ax.get_yticklabels()):
        label.set_fontname('Times New Roman')
        label.set_fontsize(20)

    ax.set_ylim(0, 1)
    # Show the plot
    ax.grid(True)
    plt.tight_layout()

    if save_name is not None:
        fig.savefig(save_name, dpi=300)
    fig.show()


def cross_product(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    return np.cross(a, b)


def closest_farthest_point_on_circle(normal, center, radius, point):
    normal = normal / np.linalg.norm(normal)  # Normalize the normal vector

    # Project point onto the plane
    to_point = point - center
    distance_to_plane = np.dot(to_point, normal)
    projection = point - distance_to_plane * normal

    direction = projection - center
    if np.linalg.norm(direction) == 0:
        direction = perpendicular_vector(normal)
    direction = direction / np.linalg.norm(direction)

    closest_point = center + radius * direction
    farthest_point = center - radius * direction

    return closest_point, farthest_point


def get_counter_clockwise_angle(v1, v2, normal=np.array([0, 0, 1])):
    # Normalize the normal vector
    normal = normal / np.linalg.norm(normal)

    with warnings.catch_warnings(record=True) as w:
        warnings.simplefilter("always")

        v1_proj = v1 - np.dot(v1, normal) * normal
        v2_proj = v2 - np.dot(v2, normal) * normal

        if np.all(v1_proj == 0) or np.all(v2_proj == 0):
            return 0

        # Normalize the projected vectors
        v1_proj = v1_proj / np.linalg.norm(v1_proj)
        v2_proj = v2_proj / np.linalg.norm(v2_proj)

        # Calculate the dot product and cross product
        dot = np.dot(v1_proj, v2_proj)
        cross = cross_product(v1_proj, v2_proj)

        # Calculate the angle using the dot product
        angle = np.arccos(np.clip(dot, -1, 1))
        if len(w) > 0:
            for warning in w:
                if issubclass(warning.category, RuntimeWarning):
                    print(f"Warning caught: {warning.message}")

    # Determine the sign of the angle using the cross product and the normal vector
    if np.dot(cross, normal) < 0:
        angle = 2 * np.pi - angle

    return angle


def distance_point_to_line(point, line_source, direction_vector):
    point = np.array(point)
    line_source = np.array(line_source)
    direction_vector = np.array(direction_vector)

    line_to_point = point - line_source

    cross = cross_product(direction_vector, line_to_point)

    distance = np.linalg.norm(cross) / np.linalg.norm(direction_vector)

    return distance


def write_tractory(file_path, coords, time_stamp, skip_list=[]):
    data = {
        'id': [i for i in range(len(coords))],
        'x[m]': [coord[0] for coord in coords],
        'y[m]': [coord[1] for coord in coords],
        'z[m]': [coord[2] for coord in coords],
        't[s]': [time_stamp for _ in coords]
    }
    df = pd.DataFrame(data)

    # skip those FLSs that are waiting
    df = df.drop(index=skip_list)

    # Check if the file exists
    file_exists = os.path.isfile(file_path)

    # Append data to CSV file (create if it doesn't exist)
    df.to_csv(file_path, mode='a', header=not file_exists, index=False)


def perpendicular_vector(normal):
    normal = np.array(normal)
    if len(normal) != 3:
        raise ValueError("The input vector must be a 3-dimensional vector.")

    # Check if the normal vector is a zero vector
    if np.linalg.norm(normal) == 0:
        raise ValueError("The input normal vector cannot be a zero vector.")

    # If the normal vector is in the z-direction
    if normal[2] != 0:
        perpendicular = np.array([1, 1, -(normal[0] + normal[1]) / normal[2]])
    # If the normal vector is in the y-direction
    elif normal[1] != 0:
        perpendicular = np.array([1, -(normal[0] + normal[2]) / normal[1], 1])
    # If the normal vector is in the x-direction
    else:
        perpendicular = np.array([-(normal[1] + normal[2]) / normal[0], 1, 1])

    return perpendicular


def load_shape(file_path, shrink_min_dist=0, shift_to_center=False, shift_bottom=None):
    points = []
    with open(file_path, 'r') as file:
        for line in file:
            # Convert each line to a list of floats
            points.append(list(map(float, line.split())))

    points = np.array(points)

    if file_path[-3:] == "xyz":
        points[:, [1, 2]] = points[:, [2, 1]]

    if shrink_min_dist > 0:
        distances = pdist(points, 'euclidean')
        min_distance = np.min(distances)
        points = points * ceil(shrink_min_dist / min_distance, 4)

    shift_coord = [0, 0, 0]
    if shift_to_center:
        shift_coord[:2] = np.mean(points[:, :2], axis=0)

    if shift_bottom is not None:
        shift_coord[2] = np.min(points[:, 2]) - shift_bottom

    points = np.array([point - shift_coord for point in points])

    return points


def log_moving_FLS(moving_FLS_num, csv_file):
    df_to_append = pd.DataFrame(moving_FLS_num)

    if os.path.exists(csv_file):
        # Read the existing CSV file into a DataFrame
        df_existing = pd.read_csv(csv_file, header=None)

        # Append the new DataFrame
        df_combined = pd.concat([df_existing, df_to_append], ignore_index=True)
    else:
        # If the CSV file does not exist, use the new DataFrame as is
        df_combined = df_to_append

    # Save the combined DataFrame to the CSV file
    df_combined.to_csv(csv_file, index=False, header=False)


def plot_line(y_lists, x, line_names, x_label, y_label, save_name=None):
    markers = ['o', '^', 's', 'D', 'h', 'v']
    colors = ["#0072BD", "#7E2F8E", "#8B6C5C", "#edb120", "#000000", "#4DBEEE"]

    fig = plt.figure(figsize=(10, 6))
    ax = fig.add_subplot(111)
    for y, line_name, color, marker in zip(y_lists, line_names, colors, markers):
        ax.plot(x, y, marker=marker, linestyle='-', color=color, label=line_name)

        ax.text(x[int(np.rint(len(x) / 2))],
                y[int(np.rint(len(x) / 2))] + (max(max((y_lists))) - min(min(y_lists))) / 20, line_name, fontsize=22,
                ha='center', va='center', color=color, font='Times New Roman')
    # Add title and labels
    ax.set_xlabel(x_label, fontsize=24, font='Times New Roman')
    title_font = {'fontname': 'Times New Roman', 'fontsize': 24}
    ax.set_title(y_label, fontdict=title_font, loc='left', pad=0)
    # ax.set_ylabel(y_label, fontsize=16, font='Times New Roman')

    # Add a legend
    # plt.legend()

    ax.spines[['right', 'top']].set_visible(False)
    # Show the plot
    ax.grid(True)
    for label in (ax.get_xticklabels() + ax.get_yticklabels()):
        label.set_fontname('Times New Roman')
        label.set_fontsize(22)

    plt.tight_layout()
    if save_name is not None:
        fig.savefig(save_name, dpi=300)
    fig.show()


def delete_file(file_path):
    try:
        os.remove(file_path)
        return f"File '{file_path}' deleted successfully."
    except FileNotFoundError:
        return f"File '{file_path}' not found."
    except Exception as e:
        return f"Error deleting file '{file_path}': {e}"


def ceil(a, precision=0):
    return np.true_divide(np.ceil(a * 10 ** precision), 10 ** precision)


def floor(a, precision=0):
    return np.true_divide(np.floor(a * 10 ** precision), 10 ** precision)


def check_collision(flss, min_distance, end_state=StateTypes.QUIT):
    collisions = []
    for i, f in enumerate(flss):
        if f.state.value >= end_state.value:
            continue

        for fls in flss[i + 1:]:
            # check if collided

            if fls.state.value >= end_state.value:
                continue

            if np.linalg.norm(np.array(fls.position) - np.array(f.position)) < min_distance:
                collisions.append([f.ID, fls.ID])

    return collisions


def find_colliding_groups(collisions):
    groups = []
    visited = set()

    def dfs(node, current_group):
        visited.add(node)
        current_group.add(node)
        for neighbor in adjacency_list[node]:
            if neighbor not in visited:
                dfs(neighbor, current_group)

    # Create adjacency list
    adjacency_list = defaultdict(list)
    for fls1, fls2 in collisions:
        adjacency_list[fls1].append(fls2)
        adjacency_list[fls2].append(fls1)

    # Find connected components (groups)
    for fls in [item for sublist in collisions for item in sublist]:
        if fls not in visited:
            current_group = set()
            dfs(fls, current_group)
            if current_group:
                groups.append(current_group)

    return groups


def report_collision_detail(groups, flss):
    collision_info = []
    for group in groups:
        info = []
        for fID in group[2]:
            fls_info = f"{flss[fID]}"
            info.append(fls_info)
        collision_info.append([group[0], group[1], len(group[2]), info])
    return collision_info


def check_collision_info(flss, collision_tracker, min_distance, end_state=StateTypes.QUIT):
    collisions = check_collision(flss, min_distance, end_state)
    groups = find_colliding_groups(collisions)
    groups = collision_tracker.update_collisions(groups)
    return report_collision_detail(groups, flss), groups


def extract_makespan(file_path):
    with open(file_path, 'r') as file:
        lines = file.readlines()  # Skip the first two lines

        title = lines[0]
        match = re.search(r"Total Steps:\s*(\d+)", title)
        makespan = int(match.group(1))

    return makespan


def extract_collision_data(file_path, target_collision_id):
    collision_data = []
    category = []

    with open(file_path, 'r') as file:
        lines = file.readlines()  # Skip the first two lines

        speeds = []

        # title = lines[0]
        # collision_num = int((title.strip('Collisions:'))[1])

        current_step = None
        # current_collision_count = 0
        for line in lines[3:]:
            line = line.strip()
            if not line:
                continue

            parts = re.split(r'[\[\]\s]+', line)

            if len(parts) == 2:  # This line indicates a new timestep
                current_step = int(parts[0])
                current_collision_count = int(parts[1])
            else:
                collision_id = int(parts[0])
                is_continuing = bool(int(parts[1]))
                num_fls = int(parts[2])

                if collision_id == target_collision_id:
                    fls_info = []
                    for i in range(num_fls):
                        fls_id = int(parts[3 + i * 8])
                        fls_status = int(parts[4 + i * 8])
                        fls_speed = float(parts[10 + i * 8])

                        if not is_continuing:
                            speeds.append(fls_speed)

                        if fls_status == StateTypes.DYN.value and fls_speed == 0:
                            fls_status = StateTypes.DYN_STAION.value

                        if not is_continuing:
                            category.append(fls_status)

                        coordinates = [float(item.replace('[', '')) for item in parts[5 + i * 8: 8 + i * 8]]
                        fls_info.append((fls_id, fls_status, coordinates))

                    collision_data.append([current_step, fls_info])

    return collision_data, category, speeds,


def draw_collision(file_path, plot_path, config, collision_FLS_nums_log, speed_log, simu_name):
    with open(file_path, 'r') as file:
        first_line = file.readline().strip()

        # Find the part after "Collisions: "
        if "Collisions: " in first_line:
            collision_count = int(first_line.split("Collisions: ")[1])
        else:
            raise ValueError("The string 'Collisions: ' was not found in the first line.")

    collision_category = []
    speed_list = []
    for ID in range(0, collision_count):
        trajectory, category, speeds = extract_collision_data(file_path, ID)
        involving_FLSs = len(trajectory[0][1])

        # plot_collision(trajectory, involving_FLSs, f"{plot_path}/{simu_name}_{ID}_{involving_FLSs}.png")
        # plot_collision_overview(trajectory, involving_FLSs, config, shape_file=f"./assets/{config.shape}.xyz",
        #                         save_name=f"{plot_path}/{simu_name}_{ID}_{involving_FLSs}_overview.png")

        collision_category.append(sorted(category))
        speed_list.extend(speeds)
    category_list = [tuple(c) for c in collision_category]

    # Use Counter to count occurrences
    category_counter = collections.Counter(category_list)

    with open(collision_FLS_nums_log, 'a') as file:
        file.write(f"{dict(category_counter)} {simu_name}\n")

    # Open the file in append mode, create it if it doesn't exist
    with open(speed_log, mode='a', newline='') as file:
        writer = csv.writer(file)

        # Write the string and list as a new row
        writer.writerow([simu_name] + speed_list)


def get_status_color(state):
    if state == StateTypes.STATIC.value:
        return 'orange'
    elif state == StateTypes.DYN.value:
        return 'blue'
    elif state == StateTypes.SYNC.value:
        return 'green'
    elif state == StateTypes.QUIT.value:
        return 'black'
    elif state == StateTypes.DYN_STAION.value:
        return 'red'
    else:
        return 'white'


def plot_collision(data, involving_FLSs, save_name, show=False):
    num_rows = len(data)
    num_columns = len(data[0])

    # Define color map (fixed face color for columns)
    colors = plt.cm.viridis(np.linspace(0, 1, involving_FLSs))

    # Define alpha values
    min_alpha = 0.2
    max_alpha = 0.8
    alpha_values = np.linspace(min_alpha, max_alpha, num_rows)

    # Create 3D plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    id_color_map = {}
    for i, row in enumerate(data):
        alpha = alpha_values[i]
        for j, element in enumerate(row[1]):
            id, status, coords = element
            x, y, z = coords

            face_color = colors[j]
            edge_color = get_status_color(status)

            ax.scatter(x, y, z, color=face_color, edgecolor=edge_color, alpha=alpha, s=100, linewidths=2)

            if id not in id_color_map:
                id_color_map[id] = face_color

    legend_elements = [plt.Line2D([0], [0], marker='o', color='w', label=f'ID {id}',
                                  markerfacecolor=color, markersize=10)
                       for id, color in id_color_map.items()]

    ax.text2D(0.05, 0.95, f"Step {data[0][0]} - Step {data[-1][0]}", transform=ax.transAxes, fontsize=12, color='black')

    # Display legend
    ax.legend(handles=legend_elements, title="Drone IDs")

    # Set plot labels
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    ax.set_aspect('equal', adjustable='box')

    if show:
        plt.show()
    if save_name is not None:
        fig.savefig(save_name, dpi=300)
        # time.sleep(0.5)
    plt.close()


def plot_collision_overview(data, involving_FLSs, config, shape_file, save_name):
    num_rows = len(data)
    # num_columns = len(data[0])
    init_formation = load_shape(shape_file, shrink_min_dist=config.Q * (2 * config.fls_size),
                                shift_to_center=True, shift_bottom=0)
    init_formation = init_formation[init_formation[:, 2].argsort()[::-1]]

    # Define color map (fixed face color for columns)
    colors = plt.cm.viridis(np.linspace(0, 1, involving_FLSs))

    # Define alpha values
    min_alpha = 0.2
    max_alpha = 0.8
    alpha_values = np.linspace(min_alpha, max_alpha, num_rows)

    # Create 3D plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    for point in init_formation:
        x, y, z = point
        ax.scatter(x, y, z, color='cyan', edgecolor='grey', alpha=0.2, s=30, linewidths=1)

    id_color_map = {}
    for i, row in enumerate(data):
        alpha = alpha_values[i]
        for j, element in enumerate(row[1]):
            id, status, coords = element
            x, y, z = coords

            face_color = colors[j]
            edge_color = get_status_color(status)

            ax.scatter(x, y, z, color=face_color, edgecolor=edge_color, alpha=alpha, s=100, linewidths=2)

            if id not in id_color_map:
                id_color_map[id] = face_color

    legend_elements = [plt.Line2D([0], [0], marker='o', color='w', label=f'ID {id}',
                                  markerfacecolor=color, markersize=10)
                       for id, color in id_color_map.items()]

    config.center[2] = np.max(init_formation[:, 2]) + max(np.max(init_formation[:, 2]) * 0.2,
                                                          (config.v_Dest ** 2) / (2 * config.acc_range[0]))
    slots = generate_points_on_circle(config.center, config.radius, config.slot_num, normal=config.normal_vector)

    x_vals = []
    y_vals = []
    z_vals = []
    # Loop through each point in the row
    for slot in slots:
        x, y, z = slot
        x_vals.append(x)
        y_vals.append(y)
        z_vals.append(z)

    ax.scatter(x_vals, y_vals, z_vals, color='green', alpha=0.5, s=100, linewidths=3)

    # Set plot labels
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    # Display legend
    ax.legend(handles=legend_elements, title="Drone IDs")

    # Set plot labels
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    # ax.set_xlim(-config.space, config.space)
    # ax.set_ylim(-config.space, config.space)
    ax.set_zlim(0, (config.center[2] + config.dist_to_opening) * 1.1)

    ax.set_aspect('equal', adjustable='box')
    # plt.show()
    if save_name is not None:
        fig.savefig(save_name, dpi=300)
        # time.sleep(0.5)

    plt.close()


def analyse_collision(config, shape_file, file_path, info_path):
    init_formation = load_shape(shape_file, shrink_min_dist=config.Q * (2 * config.fls_size),
                                shift_to_center=True, shift_bottom=0)
    init_formation = init_formation[init_formation[:, 2].argsort()[::-1]]

    config.center[2] = np.max(init_formation[:, 2]) + max(np.max(init_formation[:, 2]) * 0.2,
                                                          (config.v_Dest ** 2) / (2 * config.acc_range[0]))

    speeds = []
    collision_angle = {}
    dist = []

    with open(file_path, 'r') as file:
        lines = file.readlines()  # Skip the first two lines

        for line in lines[3:]:
            line = line.strip()
            if not line:
                continue

            dist_to_fp = []
            speed = []
            type = set()

            parts = re.split(r'[\[\]\s]+', line)

            if len(parts) == 2:  # This line indicates a new timestep
                continue
            else:
                is_continuing = bool(int(parts[1]))
                num_fls = int(parts[2])

                if not is_continuing:
                    dist_flag = True
                    flight_pattern_FLS = []
                    for i in range(num_fls):
                        fls_id = int(parts[3 + i * 8])
                        if fls_id == 2 or fls_id == 3:
                            heading = angles_to_vector(
                                [float(item.replace('[', '')) for item in parts[8 + i * 8: 10 + i * 8]])
                            coord = np.array([float(item.replace('[', '')) for item in parts[5 + i * 8: 8 + i * 8]])
                            flight_pattern_FLS.append([coord, heading])

                    for i in range(num_fls):
                        fls_id = int(parts[3 + i * 8])
                        fls_status = int(parts[4 + i * 8])
                        fls_coord = np.array([float(item.replace('[', '')) for item in parts[5 + i * 8: 8 + i * 8]])
                        fls_heading = angles_to_vector(
                            [float(item.replace('[', '')) for item in parts[8 + i * 8: 10 + i * 8]])
                        fls_speed = float(parts[10 + i * 8])

                        if fls_status != StateTypes.DYN.value:
                            dist_flag = False

                        if fls_status == StateTypes.DYN.value and flight_pattern_FLS:
                            for coord, heading in flight_pattern_FLS:
                                if np.linalg.norm(fls_coord - coord) < config.fls_size * 2:
                                    angle = get_counter_clockwise_angle(fls_heading, heading,
                                                                        normal=np.array([0, 0, 1]))
                                    if fls_id in collision_angle:
                                        (collision_angle[fls_id]).append(angle)
                                    else:
                                        collision_angle[fls_id] = [angle]
                                    break

                        speed.append(fls_speed)

                        coordinates = [float(item.replace('[', '')) for item in parts[5 + i * 8: 8 + i * 8]]

                        dist_to_fp.append(abs(coordinates[2] - config.center[2]))

                    if dist_flag:
                        dist.append(dist_to_fp)

                    speeds.append(speed)

        if collision_angle:
            with open(f'{info_path}_angle.csv', mode='w', newline='') as file:
                writer = csv.writer(file)

                for key, values in collision_angle.items():
                    writer.writerow([key] + values)

        if speeds:
            with open(f'{info_path}_speed.csv', mode='w', newline='') as file:
                writer = csv.writer(file)
                for s in speeds:
                    writer.writerow(s)

        if dist:
            with open(f'{info_path}_dist.csv', mode='w', newline='') as file:
                writer = csv.writer(file)
                for d in dist:
                    writer.writerow([statistics.mean(d)] + d)

    return twoD_list_mean(speeds), twoD_list_mean(dist)


def twoD_list_mean(data):
    total_sum = sum(sum(row) for row in data)
    num_elements = sum(len(row) for row in data)

    if num_elements == 0:
        return -1

    return total_sum / num_elements


def distance(coord1, coord2):
    return np.linalg.norm(np.array(coord1) - np.array(coord2))


def opt_schedule(flss, Lambda, opening, Salt=0):
    interArrivalTime = (1 / Lambda) * 1.1
    schedule = []
    minDepartTime = 0

    flss = sorted(flss, key=lambda f: distance(opening, f.position))

    # First loop: Calculate initial departure and arrival times
    for i, fls in enumerate(flss):
        dist = distance(opening, fls.position)

        _, _, travel_time = fls.linear_movement_OPT(dist, math.sqrt(2 * 0.2 + 0.7), float('inf'), error=0)

        arrivalT = (i - 1) * interArrivalTime
        departureT = arrivalT - travel_time

        if departureT < minDepartTime:
            minDepartTime = departureT

        event = {
            'fID': fls.ID,
            'coord': opening,
            'departureT': departureT,
            'arrivalT': arrivalT
        }
        schedule.append(event)

    # Second loop: Adjust schedule times by minDepartTime
    for i in range(len(schedule)):
        schedule[i]['departureT'] += abs(minDepartTime)
        schedule[i]['arrivalT'] += abs(minDepartTime)

    return schedule


def opt_schedule_dict(flss, Lambda, opening):
    schedule = opt_schedule(flss, Lambda, opening, 0)
    s_dict = {}

    for event in schedule:
        s_dict[event['fID']] = event

    return s_dict
