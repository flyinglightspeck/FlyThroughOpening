import csv
import re
from collections import defaultdict

# File paths
input_file = 'results/log_salt/collisions/collisions.txt'
output_file = 'results/log_salt/collisions/collision_categories.csv'

all_keys = set()
data_rows = []

pattern = re.compile(r'(\{.*?\})\s(\w+)_(\d+)_Q(\d+)_S([\d\.]+)_N_(\d+)_OPT\.txt')

# Parse the file
with open(input_file, 'r') as f:
    for line in f:
        match = pattern.match(line)
        if match:
            dict_str, name, fls_num, q_value, speed, slots = match.groups()

            current_dict = eval(dict_str)

            # Collect all keys that appear in the dictionaries
            all_keys.update(current_dict.keys())

            # Append data for the row: [name, Q, Speed, Slots, dictionary values...]
            data_rows.append([name, int(q_value), float(speed), int(slots), current_dict])

# Sort the keys for consistent column ordering
sorted_keys = sorted(all_keys, key=lambda x: len(x))
data_rows = sorted(data_rows, key=lambda x: (x[0], x[1], x[2], [3]))

# Write data to CSV
with open(output_file, 'w', newline='') as csvfile:
    # Create CSV writer
    writer = csv.writer(csvfile)

    # Write the header
    header = ['Name', 'Q', 'Speed', 'Number of Slots', 'Total Collisions'] + [str(key) for key in sorted_keys]
    writer.writerow(header)

    # Write the data rows
    for row in data_rows:
        # Fill the row with the dictionary values, using 0 for missing keys
        name, q_value, speed, slots, current_dict = row
        row_data = [name, q_value, speed, slots] + [int(sum(current_dict.values()))] + [current_dict.get(key, 0) for key in sorted_keys]
        writer.writerow(row_data)

print(f"CSV file '{output_file}' created successfully.")
