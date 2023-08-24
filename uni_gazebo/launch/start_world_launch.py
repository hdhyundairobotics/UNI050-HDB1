#!/usr/bin/python3ibgazebo_ros2_control.so" name="gazebo_ros2_control">
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from pathlib import Path

def generate_launch_description():

    gz_resource_path = SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=[
        EnvironmentVariable('GAZEBO_MODEL_PATH', default_value=''),
        '/usr/share/gazebo-11/models/:',
        str(Path(get_package_share_directory('uni_description')).parent.resolve())])

    pkg_uni_gazebo = get_package_share_directory('uni_gazebo')
    gz_world_path = os.path.join(pkg_uni_gazebo, 'worlds', 'hd_world.world')

    launch_file_dir = os.path.join(get_package_share_directory('uni_gazebo'), 'launch')

    # Launch robot state publisher
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_dir, '/robot_state_publisher.launch.py']),
    )

    # Gazebo launch
    # Gazebo server
    gzserver = ExecuteProcess(
        cmd=['gzserver',
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so',
             gz_world_path],
        output='screen',
    )

    # Gazebo client
    gzclient = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
        # condition=IfCondition(LaunchConfiguration('gui')),
    )

    # Spawn robot
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='uni_hdb1',
        arguments=['-entity',
                   'uni_hdb1',
                   '-topic',
                   'robot_description'],
        output='screen',
    )

    # Load controller
    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'uni_controller'],
        output='screen'
    )

    return LaunchDescription([
        gz_resource_path,
        gzserver,
        gzclient,
        robot_state_publisher,
        spawn_robot
    ])
