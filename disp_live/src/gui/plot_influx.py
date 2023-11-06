#!/usr/bin/env python3
import os
from influxdb_client import InfluxDBClient, QueryApi
import matplotlib.pyplot as plt
import numpy as np

# InfluxDB configuration
token = os.environ.get("INFLUXDB_TOKEN")
org = "Chung-Ang University"
url = "http://localhost:8086"
bucket = "SITL"

# Create an InfluxDB client instance
client = InfluxDBClient(url=url, token=token, org=org)

# Define the query to retrieve data for nuc3 on November 2, 2023
query = f'''
    from(bucket:"{bucket}")
      |> range(start: 2023-11-02T00:00:00Z, stop: 2023-11-03T00:00:00Z)
      |> filter(fn: (r) => r._measurement == "fiducial_transforms" and r.nuc == "nuc3")
'''

# Execute the query
query_api = client.query_api()
tables = query_api.query(query=query)

# Extract data for plotting
timestamps = []
translation_x_values = []
translation_y_values = []
rotation_z_values = []

for table in tables:
    for row in table.records:
        timestamps.append(row.values["_time"])
        translation_x_values.append(row.values["translation_x"])
        translation_y_values.append(row.values["translation_y"])
        rotation_z_values.append(row.values["rotation_z"])

# Convert timestamps to numpy datetime objects
timestamps = np.array(timestamps, dtype='datetime64[ns]')

# Create plots
plt.figure(figsize=(12, 6))

plt.subplot(2, 1, 1)
plt.plot(timestamps, translation_x_values, label='Translation X')
plt.plot(timestamps, translation_y_values, label='Translation Y')
plt.xlabel('Timestamp')
plt.ylabel('Translation Values')
plt.legend()

plt.subplot(2, 1, 2)
plt.plot(timestamps, rotation_z_values, label='Rotation Z')
plt.xlabel('Timestamp')
plt.ylabel('Rotation Z Values')
plt.legend()

plt.tight_layout()
plt.show()

# Close the InfluxDB client
client.close()
