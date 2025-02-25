import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the package directory
    package_dir = get_package_share_directory('limo_gazebo')

    # Path to the world file
    world_file = os.path.join(package_dir, 'world', 'basic.world')

    # (Optional) Set the Gazebo model path if you have custom models
    models_path = os.path.join(package_dir, 'models')
    os.environ["GAZEBO_MODEL_PATH"] = models_path

    # Start Gazebo Classic Server with ROS 2 integration plugins.
    # The following plugins are typically required:
    # - libgazebo_ros_init.so: Initializes ROS within Gazebo.
    # - libgazebo_ros_factory.so: Enables spawning of entities (robots, etc.) via ROS services.
    gazebo_server = ExecuteProcess(
        cmd=[
            'gazebo', '--verbose', world_file,
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so'
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo_server
    ])
