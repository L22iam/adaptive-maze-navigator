import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    # Paths
    world_file = os.path.expanduser('~/turtlebot3_ws/src/worlds/maze_world.sdf')
    model_path = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'models'
    )
    robot_sdf = os.path.join(model_path, 'turtlebot3_waffle', 'model.sdf')
    
    # Start Gazebo with maze world
    start_gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file],
        output='screen'
    )
    
    # Spawn robot after delay
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'turtlebot3_waffle',
            '-file', robot_sdf,
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.01'
        ],
        output='screen'
    )
    
    # ROS-Gazebo bridge for topics
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        ],
        output='screen'
    )

    return LaunchDescription([
        start_gazebo,
        spawn_robot,
        bridge
    ])
