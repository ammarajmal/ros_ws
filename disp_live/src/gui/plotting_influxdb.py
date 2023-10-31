#!/usr/bin/env python3
import influxdb_client
import matplotlib.pyplot as plt

# Connect to InfluxDB
client = influxdb_client.InfluxDBClient(url="http://localhost:8086", token="W7Chq8_KRs3zVQCD5KV1W6bTbAs3f4xoNjiUTU4JcRcv7i3uyeovQcCzIwRiyRIFLITxFlHUe_S4rQruWD6I8A==", org="Chung-Ang University")
query_api = client.query_api()

# Query data
query = 'from(bucket:"SITL") |> range(start: -1h) |> filter(fn: (r) => r._measurement == "fiducial_transforms")'
result = query_api.query(query)

# Extract data for plotting (assuming you want to plot 'translation_x' for example)
times = []
values = []
for table in result:
    for record in table.records:
        times.append(record.get_time())
        values.append(record.get_value())

# Plot using matplotlib
plt.plot(times, values)
plt.xlabel('Time')
plt.ylabel('Translation X')
plt.title('Fiducial Transforms - Translation X over Time')
plt.show()
