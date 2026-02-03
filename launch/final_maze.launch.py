import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    
    world_file = '/opt/ros/jazzy/share/turtlebot3_gazebo/worlds/maze_world.sdf'
    
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_turtlebot3_gazebo, 'launch', 'empty_world.launch.py')
        ),
        launch_arguments={'world': world_file}.items()
    )
    
    return LaunchDescription([gazebo_launch])
