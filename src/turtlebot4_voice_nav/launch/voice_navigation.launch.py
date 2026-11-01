"""Semantic navigation nodes; optionally include the previous lab's simulator."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from turtlebot4_voice_nav.map_regions import load_regions


def start(context):
    share = Path(get_package_share_directory('turtlebot4_voice_nav'))
    old_share = Path(get_package_share_directory('turtlebot4_slam_nav'))
    regions_file = LaunchConfiguration('regions_file').perform(context)
    data = load_regions(regions_file)
    nodes = [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(old_share / 'launch/navigation.launch.py')),
        condition=IfCondition(LaunchConfiguration('simulation')),
        launch_arguments={'world': 'depot', 'map': data['map_yaml'], 'rviz': 'false',
                          'gazebo_gui': LaunchConfiguration('gazebo_gui'),
                          'gui_render_engine': LaunchConfiguration('gui_render_engine')}.items())]
    params = {'regions_file': regions_file, 'use_sim_time': True}
    nodes.extend([
        Node(package='turtlebot4_voice_nav', executable='semantic_visualizer',
             parameters=[params], output='screen'),
        Node(package='turtlebot4_voice_nav', executable='voice_navigator',
             parameters=[str(share / 'config/voice_nav.yaml'), params], output='screen'),
        Node(package='rviz2', executable='rviz2', arguments=['-d', str(share / 'rviz/semantic.rviz')],
             parameters=[{'use_sim_time': True}], condition=IfCondition(LaunchConfiguration('rviz')),
             output='screen'),
    ])
    return nodes
    

def generate_launch_description():
    share = Path(get_package_share_directory('turtlebot4_voice_nav'))
    return LaunchDescription([
        DeclareLaunchArgument('simulation', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('gazebo_gui', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('gui_render_engine', default_value='ogre2',
                              choices=['ogre2', 'ogre']),
        DeclareLaunchArgument('regions_file', default_value=str(share / 'config/generated/regions.yaml')),
        OpaqueFunction(function=start),
    ])
