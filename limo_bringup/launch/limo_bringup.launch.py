#!/usr/bin/env python3

"""
This program is free software: you can redistribute it and/or modify it 
under the terms of the GNU General Public License as published by the Free Software Foundation, 
either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program. 
If not, see <https://www.gnu.org/licenses/>.

created by Thanacha Choopojcharoen at CoXsys Robotics (2022)
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os
import xacro    
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

# for display robot_state_publisher and fix something
    
def generate_launch_description():
    
    pkg_limo_gazebo = get_package_share_directory('limo_gazebo')
    pkg_limo_bringup = get_package_share_directory('limo_bringup')

    # launch sim gazebo
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    pkg_limo_gazebo,
                    "launch",
                    "sim.launch.py"
                )
            ]
        )
    )

    ackermann_steering_node = Node(
        package="limo_bringup",
        executable="ackermann_steering_node.py",
        name="ackermann_steering_node"
    )

    bicycle_model_node = Node(
        package="limo_bringup",
        executable="bicycle_model_node.py",
        name="bicycle_model_node"
    )

    odometry_calculation = Node(
        package="limo_bringup",
        executable="odometry_calculation.py",
        name="odometry_calculation"
    )



    launch_description = LaunchDescription()
    
    launch_description.add_action(sim)
    # launch_description.add_action(ackermann_steering_node)
    # launch_description.add_action(bicycle_model_node)
    # launch_description.add_action(odometry_calculation)
    
    return launch_description