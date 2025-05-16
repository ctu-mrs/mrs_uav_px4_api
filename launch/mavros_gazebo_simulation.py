#!/usr/bin/env python3

import launch
import os

from launch_ros.actions import Node, PushROSNamespace, SetParameter
from launch.actions import GroupAction, IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import (
    LaunchConfiguration,
    IfElseSubstitution,
    PythonExpression,
    PathJoinSubstitution,
    EnvironmentVariable,
)
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory


def process_launch_args(context, *args, **kwargs):
    pkg_name = "mrs_uav_px4_api"
    this_pkg_path = get_package_share_directory(pkg_name)

    id_arg = LaunchConfiguration('ID').perform(context)
    vehicle_name = LaunchConfiguration('VEHICLE_NAME').perform(context)
    fcu_url = LaunchConfiguration('fcu_url').perform(context)
    gcs_url = LaunchConfiguration('gcs_url').perform(context)


    tgt_system = int(id_arg) + 1
    uav_name = f'{vehicle_name}{id_arg}'
    namespace = uav_name

    # args from ENV
    # use_sim_time=os.getenv('USE_SIM_TIME', "false") == "true"
    respawn_mavros=os.getenv('respawn_mavros', "false") == "true"


    px4_launch_arguments = {
        "fcu_url": fcu_url,
        "gcs_url": gcs_url,
        "tgt_system": str(tgt_system),
        "tgt_component": str(1),
        "log_output": "screen",
        "fcu_protocol": "v2.0",
        "respawn_mavros": str(respawn_mavros),
        "namespace": "mavros",
        "pluginlists_yaml":  this_pkg_path + "/config/mavros_plugins.yaml",
        "config_yaml": this_pkg_path + "/config/mavros_px4_config.yaml",
    }

    print(px4_launch_arguments.items())

    # for some undiscovered reason, a python launch files crashes for mavros
    # so we use this monstrosity to start a classic launch file from a python launch file
    launch_xml_include_with_namespace = GroupAction(
        actions=[
            # push_ros_namespace first to set namespace of included nodes for following actions
            PushROSNamespace(namespace),
            IncludeLaunchDescription(
                XMLLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('mavros'),
                        'launch/node.launch')
                    ),
                    launch_arguments=px4_launch_arguments.items()
            ),
        ],
    )

    return [launch_xml_include_with_namespace]



def generate_launch_description():

    # launch.logging.get_logger().setLevel(launch.logging.logging.DEBUG)

    ld = launch.LaunchDescription()

    # #{ custom_config

    custom_config = LaunchConfiguration('custom_config')

    # this adds the args to the list of args available for this launch files
    # these args can be listed at runtime using -s flag
    # default_value is required to if the arg is supposed to be optional at launch time
    ld.add_action(DeclareLaunchArgument(
        'custom_config',
        default_value="",
        description="Path to the custom configuration file. The path can be absolute, starting with '/' or relative to the current working directory",
    ))

    # behaviour:
    #     custom_config == "" => custom_config: ""
    #     custom_config == "/<path>" => custom_config: "/<path>"
    #     custom_config == "<path>" => custom_config: "$(pwd)/<path>"
    custom_config = IfElseSubstitution(
            condition=PythonExpression(['"', custom_config, '" != "" and ', 'not "', custom_config, '".startswith("/")']),
            if_value=PathJoinSubstitution([EnvironmentVariable('PWD'), custom_config]),
            else_value=custom_config
    )

    # #} end of custom_config

    # #{ args from mrs_drone_spawner
    ld.add_action(DeclareLaunchArgument(
        'ID',
        default_value="1",
        description='ID number assigned to the vehicle by the drone spawner'
    ))
    ld.add_action(DeclareLaunchArgument(
        'VEHICLE_NAME',
        default_value="uav",
        description='prefix for the vehicle name, full name will be {VEHICLE_NAME}{ID}'
    ))
    ld.add_action(DeclareLaunchArgument(
        'fcu_url',
        default_value=f"udp://:14541@127.0.0.1:14581",
        description='port setup for the fcu'
    ))
    ld.add_action(DeclareLaunchArgument(
        'gcs_url',
        default_value=f"udp://:14551@127.0.0.1:14591",
        description='port setup for the gcs'
    ))
    # #}


    # this needs to be done whenever we want to parse and modify launch arguments
    # directly in the launch file
    ld.add_action(OpaqueFunction(function=process_launch_args))

    return ld


