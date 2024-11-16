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
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    ld = LaunchDescription()
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_file = os.path.join(get_package_share_directory('reinforcement_learning_drive'),
                'world',
                'empty.world')

    bridge_node = Node(
        package='reinforcement_learning_drive',
        executable='reinforcement_learning_drive_node',
        name='reinforcement_learning_drive_node',
        parameters=[{
            'use_sim_time': True,
            'num_actors': 4
        }],
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

    gazebo_launch_file = os.path.join(
        get_package_share_directory('gazebo_ros'),
        'launch',
        'gazebo.launch.py',
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_file),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'world': world_file
        }.items()
    )

    # finalize
    ld.add_action(rviz_node)
    ld.add_action(static_transform_publisher_node)
    ld.add_action(bridge_node)
    ld.add_action(gazebo_launch)

    return ld