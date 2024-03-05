#!/usr/bin/env python3


# export INFLUXDB_TOKEN="iWhd49JVf1sj4D5DIvTys2pAvNLRTag0tCSO9VZj2MMFML5j6gtTLY6R6cZs0Dq7rrwHCB3O0pJLJpWVQaQnew=="


# w-oBxVq-vThtv1pHtTozECZtlglpGYg4cHmU0_BtcVrR1anMSU0ZV1Iu3GoMEZBGqQDbdi7iRfvTB9iQJ_k95Q==

import os

token = os.environ.get("INFLUXDB_TOKEN")

if token:
    # Use the token in your InfluxDB-related code here
    print(f"InfluxDB Token: {token}")
else:
    print("INFLUXDB_TOKEN environment variable not set.")

