#!/bin/bash
# Get the IP address of the current machine
ip_address=$(ifconfig | grep "inet " | grep -v 127.0.0.1 | awk '{print $2}' | head -n 1)
echo "The IP address is: $ip_address"
