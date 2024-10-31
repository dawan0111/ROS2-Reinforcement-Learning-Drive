# MIT License

# Copyright (c) 2020 Hongrui Zheng

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    ld = LaunchDescription()

    map_yaml_file = os.path.join(get_package_share_directory('reinforcement_learning_drive'),
                'map',
                'map.yaml')

    bridge_node = Node(
        package='reinforcement_learning_drive',
        executable='reinforcement_learning_drive_node',
        name='reinforcement_learning_drive_node',
        parameters=[{'use_sim_time': True}]
    )

    static_transform_publisher_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher_map_to_odom",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        # arguments=['-d', os.path.join(get_package_share_directory('f1tenth_gym_ros'), 'launch', 'gym_bridge.rviz')]
    )
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        parameters=[{'yaml_filename': map_yaml_file},
                    {'topic': 'map'},
                    {'frame_id': 'map'},
                    {'output': 'screen'},
                    {'use_sim_time': True}]
    )
    nav_lifecycle_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'use_sim_time': True},
                    {'autostart': True},
                    {'node_names': ['map_server']}]
    )

    actor_launch_file = os.path.join(
        get_package_share_directory('ros2_control_demo_example_11'),
        'launch',
        'carlikebot.launch.py'
    )

    actor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(actor_launch_file),
    )

    # finalize
    ld.add_action(rviz_node)
    ld.add_action(nav_lifecycle_node)
    ld.add_action(map_server_node)
    ld.add_action(static_transform_publisher_node)
    ld.add_action(bridge_node)
    # ld.add_action(actor_launch)

    return ld