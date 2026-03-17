from launch import LaunchDescription
from launch_ros.actions import Node
import subprocess
from launch.actions import ExecuteProcess
import re
import os

def get_ip_from_mac(mac_addr):
    """
    Queries the ARP table via `ip neigh` to find the IP address corresponding to the given MAC.
    Returns the first matching IP, or None if not found.
    """
    try:
        # Execute the "ip neigh show" command
        output = subprocess.check_output(['ip', 'neigh', 'show']).decode('utf-8')
    except subprocess.CalledProcessError:
        return None

    pattern = re.compile(r'(?P<ip>\d+\.\d+\.\d+\.\d+)\s+dev\s+\S+\s+lladdr\s+' + re.escape(mac_addr), re.IGNORECASE)
    for line in output.splitlines():
        m = pattern.search(line)
        if m:
            return m.group('ip')
    return None

def generate_launch_description():
    ld = LaunchDescription()

    ip_drone_1 = '192.168.1.2'
    
    drones = [
        {'namespace': 'drone_1', 'ip_rc': ip_drone_1}
    ]
    
    # Add RTSP streaming node

    drone = drones[0]
    
    rtsp_node = Node(
        
        package='drone_videofeed',
        executable='rtsp_node',
        namespace=drone['namespace'],
        parameters=[{'ip_rc': drone['ip_rc']}]
    )
    ld.add_action(rtsp_node)

    # Add drone nodes dynamically
    for drone in drones:
        node = Node(
            package='dji_controller',
            executable='dji_node',
            namespace=drone['namespace'],
            parameters=[{'ip_rc': drone['ip_rc']}],
        )
        ld.add_action(node)
        
    return ld
