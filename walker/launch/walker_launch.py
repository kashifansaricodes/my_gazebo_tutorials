from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the turtlebot3 gazebo package directory
    pkg_gazebo_ros = get_package_share_directory('turtlebot3_gazebo')
    
    # Set Gazebo model path
    gazebo_model_path = os.path.join(pkg_gazebo_ros, 'models')
    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] += ":" + gazebo_model_path
    else:
        os.environ['GAZEBO_MODEL_PATH'] = gazebo_model_path

    # Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        )
    )

    # Your walker node
    walker_node = Node(
        package='walker',
        executable='walker_node',
        name='walker_node',
        output='screen'
    )

    return LaunchDescription([
        # Set env var for Gazebo
        ExecuteProcess(
            cmd=['bash', '-c', 'export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH'],
            output='screen'
        ),
        # Launch Gazebo
        gazebo,
        # Your walker node
        walker_node
    ])