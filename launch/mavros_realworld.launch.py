#!/usr/bin/env python3

import launch
import os

from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

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

    tgt_system = 1
    namespace = uav_name

    # the mavlink_router <-> uas bridge topics are global (not pushed under the node
    # namespace) by mavros' design, so give each vehicle its own prefix here instead of
    # relying on mavros_node's default of "/uas<tgt_system>", which collides across vehicles
    # that share tgt_system and pollutes the global topic namespace
    uas_url = f'/{uav_name}/mavlink'

    ld.add_action(ComposableNodeContainer(

        namespace=namespace,
        name=namespace + '_mavros_container',
        package='rclcpp_components',
        executable='component_container_events_cbg',
        output='screen',

        composable_node_descriptions=[

            ComposableNode(

                package='mavros',
                plugin='mavros::router::Router',
                namespace=namespace + '/mavros',
                name='mavros_router',
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
                name='mavros',
                parameters=[

                    {'uas_url': uas_url},
                    {'target_system_id': tgt_system},
                    {'target_component_id': 1},
                    {'fcu_protocol': 'v2.0'},
                    this_pkg_path + '/config/mavros_plugins.yaml',
                    this_pkg_path + f'/config/mavros_px4_config{("_old_fw" if OLD_PX4_FW else "")}.yaml',

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
