#!/usr/bin/env python3
"""
Launch file for running rm_decision behavior tree with simulation data.
This launches both the decision tree and the simulation node to test the behavior.
"""

import os.path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Get config file path
    config = os.path.join(
        get_package_share_directory('rm_decision'), 'config', 'node_params.yaml')
    
    # Decision tree node
    decision_node = Node(
        package='rm_decision',
        name='rm_decision_tree_exec',
        executable='tree_exec_node',
        parameters=[config],
        arguments=['--ros-args', '--log-level', 'info'],
        output='screen')
    
    # Simulation node
    simulation_node = Node(
        package='rm_decision',
        name='simulation_node',
        executable='simulation_node',
        output='screen')
    
    ld = LaunchDescription()
    ld.add_action(decision_node)
    ld.add_action(simulation_node)
    
    return ld
