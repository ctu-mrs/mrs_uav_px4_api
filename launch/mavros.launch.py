#!/usr/bin/env python3

import launch
import os

from launch.actions import DeclareLaunchArgument
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    ld = launch.LaunchDescription()

    pkg_name = 'mrs_uav_px4_api'

    this_pkg_path = get_package_share_directory(pkg_name)

    # #{ args from ENV

    UAV_NAME = LaunchConfiguration('uav_name')

    fcu_url = LaunchConfiguration('fcu_url')
    config_yaml = LaunchConfiguration('config_yaml')
    use_sim_time = LaunchConfiguration('use_sim_time')

    OLD_PX4_FW = os.getenv('OLD_PX4_FW', 'false') == 'true'
    PX4_IP = os.getenv('PX4_IP', '')

    if PX4_IP:
        default_fcu_url = f'udp://:14550@{PX4_IP}:14550'
    else:
        rate = 921600 if OLD_PX4_FW else 2000000
        default_fcu_url = f'/dev/pixhawk:{rate}'

    ld.add_action(DeclareLaunchArgument(
        'uav_name',
        default_value=os.getenv('UAV_NAME', 'uav'),
        description='UAV namespace used by MAVROS',
    ))

    ld.add_action(DeclareLaunchArgument(
        'fcu_url',
        default_value=default_fcu_url,
        description='FCU URL used by MAVROS',
    ))

    ld.add_action(DeclareLaunchArgument(
        'use_sim_time',
        default_value=os.getenv('USE_SIM_TIME', 'false'),
        description='Whether MAVROS should use sim time',
    ))

    ld.add_action(DeclareLaunchArgument(
        'config_yaml',
        default_value=os.path.join(this_pkg_path, f'/config/mavros_px4_config{("_old_fw" if OLD_PX4_FW else "")}.yaml'),
        description='Path to the MAVROS PX4 config YAML file',
    ))

    # gcs_url = 'tcp-l://'
    gcs_url = ''  # do not connect to QGC using mavros, we create a dedicated mavlink stream instead

    # #} end of args from ENV

    ld.add_action(Node(
        package='mavros',
        executable='mavros_node',
        namespace=[UAV_NAME, '/mavros'],
        output='screen',
        parameters=[
            {"fcu_url": fcu_url},
            {"gcs_url": gcs_url},
            {"tgt_system": 1},
            {"tgt_component": 1},
            {"fcu_protocol": 'v2.0'},
            {"use_sim_time": use_sim_time},

            {"base_link_frame_id": [UAV_NAME, '/base_link']},
            {"odom_frame_id": [UAV_NAME, '/odom']},
            {"map_frame_id": [UAV_NAME, '/map']},

            {"pluginlists_yaml": this_pkg_path + '/config/mavros_plugins.yaml'},
            {"config_yaml": config_yaml},
        ],
        remappings=[
            ('/diagnostics', 'diagnostics'),
            ('/uas1/mavlink_source', 'mavlink_source'),
            ('/uas1/mavlink_sink', 'mavlink_sink'),
        ]
    ))

    ld.add_action(
        Node(
            package='tf2_ros',
            namespace='',
            executable='static_transform_publisher',
            name='fcu_to_garmin',
            arguments=['0.0', '0.0', '-0.05', '0', '1.57', '0', [UAV_NAME, '/fcu'], 'garmin'],
        )
    )

    return ld
