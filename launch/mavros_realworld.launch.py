#!/usr/bin/env python3

import launch
import os

from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterFile

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    ld = launch.LaunchDescription()

    pkg_name = 'mrs_uav_px4_api'

    this_pkg_path = get_package_share_directory(pkg_name)

    # #{ args from ENV

    uav_name = os.getenv('UAV_NAME', 'uav')
    OLD_PX4_FW = os.getenv('OLD_PX4_FW', 'false') == 'true'
    PX4_IP = os.getenv('PX4_IP', '')

    if PX4_IP:
      fcu_url = f'udp://:14550@{PX4_IP}:14550'
    else:
      rate = 921600 if OLD_PX4_FW else 2000000
      fcu_url = f'/dev/pixhawk:{rate}'

    gcs_url = 'tcp-l://'

    # #} end of args from ENV

    namespace = uav_name
    uas_url = f'/{uav_name}/mavlink'

    ld.add_action(ComposableNodeContainer(

        namespace=namespace,
        name='mavros',
        package='rclcpp_components',
        executable='component_container_events_cbg',
        output='screen',

        composable_node_descriptions=[

            ComposableNode(

                package='mavros',
                plugin='mavros::router::Router',
                namespace=namespace + '/mavros',
                name='router',
                parameters=[

                    {'fcu_urls': [fcu_url]},
                    {'gcs_urls': [gcs_url]},
                    {'uas_urls': [uas_url]},

                ],
            ),

            ComposableNode(

                package='mavros',
                plugin='mavros::uas::UAS',
                namespace=namespace + '/mavros',
                name='uas',
                parameters=[

                    {'uas_url': uas_url},
                    {'target_system_id': 1},
                    {'target_component_id': 1},
                    {'fcu_protocol': 'v2.0'},
                    ParameterFile(this_pkg_path + '/config/mavros_plugins.yaml', allow_substs=True),
                    ParameterFile(this_pkg_path + f'/config/mavros_px4_config{("_old_fw" if OLD_PX4_FW else "")}.yaml', allow_substs=True),

                ],
            ),

        ],

    ))

    ld.add_action(
        Node(
            package='tf2_ros',
            namespace='',
            executable='static_transform_publisher',
            name='fcu_to_garmin',
            arguments=['0.0', '0.0', '-0.05', '0', '1.57', '0', uav_name + '/fcu', 'garmin'],
        )
    )

    return ld
