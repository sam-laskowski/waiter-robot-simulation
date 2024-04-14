import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

from time import sleep

def generate_launch_description():
    tb3_gazebo_launch_file_dir = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_tb3_sim = get_package_share_directory('tb3_sim')


    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    z_pose = LaunchConfiguration('z_pose', default='0.0')

    world = os.path.join(
      get_package_share_directory('turtlebot3_gazebo'),
      'worlds',
      'restaurant3_final.world'
    )

    gzserver_cmd = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
          os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
      ),
      launch_arguments={'world': world}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
          os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
      )
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
          os.path.join(tb3_gazebo_launch_file_dir, 'robot_state_publisher.launch.py')
      ),
      launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    spawn_turtlebot_cmd = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
          os.path.join(tb3_gazebo_launch_file_dir, 'spawn_turtlebot3.launch.py')
      ),
      launch_arguments={
          'x_pose': x_pose,
          'y_pose': y_pose,
          'z_pose': z_pose,
      }.items()
    )


    pkg_nav2_dir = get_package_share_directory('nav2_bringup')
    pkg_tb3_sim = get_package_share_directory('tb3_sim')

    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    autostart = LaunchConfiguration('autostart', default='True')

    nav2_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'map': os.path.join(pkg_tb3_sim, 'maps', 'restaurant3_map2.yaml')
        }.items()
    )


    rviz_launch_cmd = Node(
      package="rviz2",
      executable="rviz2",
      name="rviz2",
      arguments=[
          '-d' + os.path.join(
              get_package_share_directory('nav2_bringup'),
              'rviz',
              'nav2_default_view.rviz'
          )]
    )

    set_init_amcl_pose_cmd = Node(
      package="tb3_sim",
      executable="amcl_init_pose_publisher",
      name="amcl_init_pose_publisher",
      parameters=[{
          "x": 0.0,
          "y": 0.0,
      }]
    )

    ld = LaunchDescription()

    # Add the commands to the launch description


    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_turtlebot_cmd)

    ld.add_action(nav2_launch_cmd)
    #ld.add_action(set_init_amcl_pose_cmd)
    #ld.add_action(rviz_launch_cmd)
    return ld