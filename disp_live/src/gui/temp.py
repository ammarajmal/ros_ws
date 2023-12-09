#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# import socket

# def get_lan_ip():
#     """ Get the local IP address of the machine """
#     s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#     try:
#         # doesn't even have to be reachable
#         s.connect(('8.8.8.8', 1))
#         ip_addr = s.getsockname()[0]
#     except socket.error:
#         ip_addr = '127.0.0.1'
#     finally:
#         s.close()
#     return ip_addr

# lan_ip = get_lan_ip()
# print("LAN IP Address:", lan_ip)
import subprocess

def disable_network_manager_for_interface(interface):
    config = f"keyfile\nunmanaged-devices=interface-name:{interface}\n"
    try:
        with open('/etc/NetworkManager/conf.d/99-disable-interface.conf', 'w') as file:
            file.write(config)
        subprocess.run(['sudo', 'systemctl', 'restart', 'NetworkManager'], check=True)
        print(f"Network Manager control disabled for {interface}.")
    except Exception as e:
        print(f"An error occurred: {e}")

disable_network_manager_for_interface('enp3s0')

from pyroute2 import IPRoute
import socket

def set_ip_address(interface, ip_address, netmask):
    ip = IPRoute()

    try:
        # Find the index of the network interface
        idx_list = ip.link_lookup(ifname=interface)
        if not idx_list:
            print(f"No interface found with the name {interface}.")
            return
        idx = idx_list[0]

        # First, bring down the interface to flush the addresses
        ip.link('set', index=idx, state='down')

        # Flush the IP addresses on the interface
        ip.flush_addr(index=idx, family=socket.AF_INET)

        # Add the new IP address
        ip.addr('add', index=idx, address=ip_address, mask=netmask, family=socket.AF_INET)

        # Bring up the interface
        ip.link('set', index=idx, state='up')
        print(f"IP address set to {ip_address} with netmask {netmask} on {interface}.")
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        ip.close()

set_ip_address('enp3s0', '192.168.0.10', 24)  # 24 is the CIDR notation for the netmask 255.255.255.0


