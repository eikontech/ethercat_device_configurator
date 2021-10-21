import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    device_configurator_share_dir = get_package_share_directory('ethercat_device_configurator')

    namespace_value = LaunchConfiguration('namespace', default='/')
    device_configurator_config_path = LaunchConfiguration(
        'device_configurator_config_path', default=device_configurator_share_dir + '/config/my_setup.yaml')

    logger_name = LaunchConfiguration("log_level", default="info")

    device_configurator_cmd = Node(
        package='ethercat_device_configurator',
        executable='standalone',
        name='ethercat_device_configurator',
        output='screen',
        namespace=namespace_value,
        arguments=[device_configurator_config_path, '--ros-args', '--log-level', logger_name]
        )

    ld = LaunchDescription()

    ld.add_action(device_configurator_cmd)

    return ld
