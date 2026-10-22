"""Classroom world + animated students + proctor robot + ROS bridge + patrol + camera viewer.

ros2 launch exam_proctor_sim proctor_sim.launch.py [headless:=true] [patrol:=false]
    [camera_view:=false] [patrol_strategy:=coverage|random] [patrol_seed:=-1]
    [students:=false] [cheat_probability:=0.1] [student_seed:=-1]
    [x:=.. y:=.. yaw:=..] [cam_width:=.. cam_height:=.. cam_fps:=..]
"""
import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('exam_proctor_sim')
    world = os.path.join(pkg, 'worlds', 'classroom.sdf')
    xacro_file = os.path.join(pkg, 'urdf', 'proctor_bot.urdf.xacro')
    bridge_config = os.path.join(pkg, 'config', 'bridge.yaml')
    gui_config = os.path.join(pkg, 'config', 'gui.config')
    arg = LaunchConfiguration

    args = [
        DeclareLaunchArgument('headless', default_value='false', description='Run gz server only'),
        DeclareLaunchArgument('patrol', default_value='true', description='Start the patrol node'),
        DeclareLaunchArgument('camera_view', default_value='true',
                              description='Open rqt_image_view on the camera feed'),
        DeclareLaunchArgument('patrol_strategy', default_value='coverage',
                              description="'coverage': randomised, biased to unseen students; 'random': plain random walk and gaze"),
        DeclareLaunchArgument('patrol_seed', default_value='-1',
                              description='Random seed for the patrol route and gaze (-1: random)'),
        DeclareLaunchArgument('students', default_value='true',
                              description='Animate students and publish ground truth'),
        DeclareLaunchArgument('cheat_probability', default_value='0.1',
                              description='Chance that a new student episode is cheating'),
        DeclareLaunchArgument('student_seed', default_value='-1',
                              description='Random seed for student behaviour (-1: random)'),
        # Default spawn: front corridor, facing the class, on an aisle-graph node.
        DeclareLaunchArgument('x', default_value='3.8'),
        DeclareLaunchArgument('y', default_value='1.8'),
        DeclareLaunchArgument('yaw', default_value='3.14159'),
        DeclareLaunchArgument('cam_width', default_value='1280'),
        DeclareLaunchArgument('cam_height', default_value='720'),
        DeclareLaunchArgument('cam_fps', default_value='15'),
    ]

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' cam_width:=', arg('cam_width'),
                 ' cam_height:=', arg('cam_height'), ' cam_fps:=', arg('cam_fps')]),
        value_type=str)

    # Where Gazebo finds the StudentAnimator system.
    plugin_path = AppendEnvironmentVariable(
        'GZ_SIM_SYSTEM_PLUGIN_PATH', os.path.join(get_package_prefix('exam_proctor_gz_plugins'), 'lib'))

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': ['-r -v 2 ', world,
                        PythonExpression(["' -s' if '", arg('headless'), "' == 'true' else ''"]),
                        ' --gui-config ', gui_config],
            'on_exit_shutdown': 'true',
        }.items())

    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        output='screen')

    spawn = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-topic', 'robot_description', '-name', 'proctor_bot',
                   '-x', arg('x'), '-y', arg('y'), '-z', '0.02', '-Y', arg('yaw')],
        output='screen')

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        parameters=[{'config_file': bridge_config, 'use_sim_time': True}],
        output='screen')

    patrol = Node(
        package='exam_proctor_sim', executable='patrol_node',
        parameters=[{
            'use_sim_time': True,
            'strategy': arg('patrol_strategy'),
            'seed': ParameterValue(arg('patrol_seed'), value_type=int),
        }],
        condition=IfCondition(arg('patrol')),
        output='screen')

    students = Node(
        package='exam_proctor_sim', executable='student_behaviors',
        parameters=[{
            'use_sim_time': True,
            'cheat_probability': ParameterValue(arg('cheat_probability'), value_type=float),
            'seed': ParameterValue(arg('student_seed'), value_type=int),
        }],
        condition=IfCondition(arg('students')),
        output='screen')

    camera_view = Node(
        package='rqt_image_view', executable='rqt_image_view',
        arguments=['/proctor/camera/image_raw'],
        condition=IfCondition(arg('camera_view')))

    return LaunchDescription(
        args + [plugin_path, gazebo, robot_state_publisher, spawn, bridge, patrol, students, camera_view])
