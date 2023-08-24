import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import xacro

def generate_launch_description():

  use_sim_time = LaunchConfiguration('use_sim_time', default='false')

  pkg_uni_description = get_package_share_directory('uni_description')
  xacro_file = os.path.join(pkg_uni_description, 'urdf/', 'uni_base_robot.urdf.xacro')
  assert os.path.exists(xacro_file), "Xacro file doesn't exist in "+str(xacro_file)

  robot_description_config = xacro.process_file(xacro_file)
  robot_desc = robot_description_config.toxml()

  robot_description_cmd = [
      'xacro',
      ' ',
      '--inorder',
      xacro_file
  ]

  robot_description = ExecuteProcess(
      cmd=['bash', '-c', ' '.join(robot_description_cmd)],
      # output='screen'
  )


  return LaunchDescription([
        robot_description,
          Node(
              package='robot_state_publisher',
              executable='robot_state_publisher',
              name='robot_state_publisher',
              output='screen',
              parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
          )
  ])

