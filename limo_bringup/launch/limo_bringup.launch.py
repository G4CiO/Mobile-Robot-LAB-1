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
from launch.substitutions import LaunchConfiguration

# for display robot_state_publisher and fix something
    
def generate_launch_description():
    
    pkg_limo_gazebo = get_package_share_directory('limo_gazebo')
    pkg_limo_bringup = get_package_share_directory('limo_bringup')
    pkg_limo_localization = get_package_share_directory('limo_localization')

    # Declare launch argument for steering mode
    steering_mode_arg = DeclareLaunchArgument(
        'steering_mode',
        default_value='ackermann',
        description='Select steering mode: "ackermann" or "bicycle"'
    )

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

    # Steering model node with configurable steering mode
    steering_model_node = Node(
        package="limo_bringup",
        executable="steering_model_node.py",
        name="steering_model_node",
        parameters=[{"steering_mode": LaunchConfiguration("steering_mode")}]
    )

    odometry_calculation = Node(
        package="limo_bringup",
        executable="odometry_calculation.py",
        name="odometry_calculation"
    )

    controller_server = Node(
        package="limo_controller",
        executable="controller_server.py",
        name="controller_server"
    )

    path_publisher = Node(
        package="limo_controller",
        executable="path_publisher.py",
        name="path_publisher"
    )
    
    fake_gps_node = Node(
    package="limo_bringup",
    executable="fake_gps_node.py",
    name="fake_gps_node",
    output="screen"
    )
    
     # Get path to the EKF configuration file
    ekf_config_path = os.path.join(
        pkg_limo_localization,
        'config',
        'ekf.yaml'
    )
    
     # EKF node (using robot_localization)
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path]
    )

    launch_description = LaunchDescription()
    launch_description.add_action(steering_mode_arg)
    launch_description.add_action(sim)
    launch_description.add_action(steering_model_node)
    launch_description.add_action(odometry_calculation)
    launch_description.add_action(fake_gps_node)
    # launch_description.add_action(ekf_node)
    # launch_description.add_action(controller_server)
    # launch_description.add_action(path_publisher)
    
    return launch_description