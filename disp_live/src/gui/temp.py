#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import subprocess
import rospy
import re

def get_ros_topic_frequency(topic):
    # Start the rostopic hz command
    process = subprocess.Popen(['rostopic', 'hz', topic], stdout=subprocess.PIPE, stderr=subprocess.STDOUT)

    # Wait a bit to get some output
    rospy.sleep(0.9)

    # Terminate the process
    process.terminate()

    # Read the output
    try:
        output, _ = process.communicate(timeout=2)
        output = output.decode('utf-8')
    except subprocess.TimeoutExpired:
        process.kill()
        output, _ = process.communicate()

    # Extract frequency from the output using regular expression
    match = re.search(r'average rate: ([\d\.]+)', output)
    if match:
        return float(match.group(1))
    else:
        return None

# Example usage
frequency = get_ros_topic_frequency('/nuc2/image_raw')
if frequency is not None:
    print(f"Frequency of /nuc2/image_raw: {frequency} Hz")
else:
    print("Unable to determine the frequency.")
