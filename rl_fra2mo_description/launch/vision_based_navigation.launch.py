from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    fra2mo_dir = FindPackageShare('rl_fra2mo_description')
    nav2_bringup_dir = FindPackageShare('nav2_bringup')
    aruco_ros_dir = FindPackageShare('aruco_ros')

    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([fra2mo_dir, 'config', 'explore.yaml']),
        description='Full path to the ROS2 parameters file to use for all launched nodes',
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use simulation (Gazebo) clock if true'
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([fra2mo_dir, 'launch', 'fra2mo_slam.launch.py'])
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([nav2_bringup_dir, 'launch', 'navigation_launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
        }.items(),
    )


    aruco_ros_node = Node(
        package='aruco_ros',  
        executable='single',  
        name='aruco_single',
        output='screen',
        parameters=[
            {'marker_size': 0.1},
            {'marker_id': 115},
            {'reference_frame': 'camera_link_optical'},
            {'camera_frame': 'camera_link_optical'},
            {'marker_frame': 'aruco_marker_frame'},
            {'image_is_rectified': True},
            {'use_sim_time': use_sim_time},
        ],
        remappings=[
            ('/image', '/videocamera'),
            ('/camera_info', '/camera_info'),
        ],
    )

    return LaunchDescription(
        [
            declare_params_file_cmd,
            declare_use_sim_time_cmd,
            slam_launch,
            nav2_bringup_launch,
            aruco_ros_node, 
        ]
    )
