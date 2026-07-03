#!/usr/bin/env python3

import launch
import os

from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.conditions import IfCondition 
from launch_ros.parameter_descriptions import ParameterFile
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    ld = launch.LaunchDescription()

    pkg_name = 'mrs_uav_px4_api'

    this_pkg_path = get_package_share_directory(pkg_name)

    # #{ args from ENV

    UAV_NAME = LaunchConfiguration('uav_name')

    fcu_url = LaunchConfiguration('fcu_url')
    tgt_system = LaunchConfiguration('tgt_system')
    config_yaml = LaunchConfiguration('config_yaml')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_default_garmin_tf = LaunchConfiguration('use_default_garmin_tf')

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
        default_value=os.path.join(this_pkg_path, 'config', f'mavros_px4_config{("_old_fw" if OLD_PX4_FW else "")}.yaml'),
        description='Path to the MAVROS PX4 config YAML file',
    ))

    ld.add_action(DeclareLaunchArgument(
        'tgt_system',
        default_value="1",
        description='Target system ID for MAVROS',
    ))

    ld.add_action(DeclareLaunchArgument(
        'use_default_garmin_tf',
        default_value='true',
        description='Whether to use the default Garmin TF transform',
    ))

    gcs_url = 'tcp-l://'

    # #} end of args from ENV

    ld.add_action(Node(
        package='mavros',
        executable='mavros_node',
        namespace=[UAV_NAME, '/mavros'],
        output='screen',
        parameters=[
            {"fcu_url": fcu_url},
            {"gcs_url": gcs_url},
            {"tgt_system": tgt_system},
            {"tgt_component": 1},
            {"fcu_protocol": 'v2.0'},
            {"use_sim_time": use_sim_time},

            {"base_link_frame_id": [UAV_NAME, '/base_link']},
            {"odom_frame_id": [UAV_NAME, '/odom']},
            {"map_frame_id": [UAV_NAME, '/map']},

            ParameterFile(this_pkg_path + '/config/mavros_plugins.yaml', allow_substs=True),
            ParameterFile(config_yaml, allow_substs=True),
        ],
        remappings=[
            ('/diagnostics', 'diagnostics'),
            (['/uas', tgt_system, '/mavlink_source'], 'mavlink_source'),
            (['/uas', tgt_system, '/mavlink_sink'], 'mavlink_sink'),
        ]
    ))

    ld.add_action(
        Node(
            package='tf2_ros',
            namespace='',
            executable='static_transform_publisher',
            name='fcu_to_garmin',
            arguments=['0.0', '0.0625', '-0.009', '0', '1.5708', '-1.5708', [UAV_NAME, '/fcu'], [UAV_NAME, '/garmin']],
            condition=IfCondition(use_default_garmin_tf)
        )
    )

    return ld
