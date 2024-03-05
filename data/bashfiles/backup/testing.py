#!/usr/bin/env python3

file1 = 'cameras_1_20s_2023-05-22_14-48-34.bag'
file2 = 'cameras_12_20s_2023-05-22_14-48-34.bag'
file3 = 'cameras_123_20s_2023-05-22_14-48-34.bag'
print(file1)

# Now I wanted to save '1' or '12' or '123' in a variable called camera_number
import re

def parse_file_name(file_name):
    if match := re.search(
        r'cameras_(\d+)_([\d]+)s_(\d{4}-\d{2}-\d{2})_(\d{2}-\d{2}-\d{2})',
        file_name,
    ):
        cam_num = match[1]
        duration = f'{match[2]}s'
        date_var = match[3].replace('-', '_')
        time_var = match[4]

        # Display the variables
        print("cam_num:", cam_num)
        print("duration:", duration)
        print("date_var:", date_var)
        print("time_var:", time_var)
    else:
        print("Invalid file name.")

# Test the function with the provided file names
file1 = 'cameras_1_10s_2023-05-22_14-48-34.bag'
file2 = 'cameras_12_20s_2023-05-22_14-48-34.bag'
file3 = 'cameras_123_30s_2023-05-22_14-48-34.bag'

parse_file_name(file1)
parse_file_name(file2)
parse_file_name(file3)
