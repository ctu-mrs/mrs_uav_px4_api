#!/usr/bin/env python3

import launch
import os
import launch_ros

import jinja2
import tempfile
import datetime

from launch_ros.actions import PushROSNamespace
from launch.actions import GroupAction, IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import (
    LaunchConfiguration,
    IfElseSubstitution,
    PythonExpression,
    PathJoinSubstitution,
    EnvironmentVariable,
)
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory, get_package_share_path


PKG_NAME = 'mrs_uav_px4_api'

# #{ generate_mavros_config(context)
def generate_mavros_config(context):
    
    resource_path = os.path.join(get_package_share_path(PKG_NAME), 'config')
    jinja_env = jinja2.Environment(
            loader=jinja2.FileSystemLoader(resource_path),
            autoescape=False)
    template = jinja_env.get_template('mavros_px4_config.jinja.yaml')
    rendered_template = template.render(uav_name=uav_name)

    fd, filepath = tempfile.mkstemp(prefix='mavros_px4_config_' + str(uav_name) + datetime.datetime.now().strftime("_%Y_%m_%d__%H_%M_%S"), suffix='.yaml')
    with os.fdopen(fd, 'w') as output_file:
        output_file.write(rendered_template)

    return [filepath]
# #}

def generate_launch_description():

    # launch.logging.get_logger().setLevel(launch.logging.logging.DEBUG)

    ld = launch.LaunchDescription()

    this_pkg_path = get_package_share_directory(PKG_NAME)

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

    # #{ declare args
    ID = LaunchConfiguration('ID')
    ld.add_action(DeclareLaunchArgument(
        'ID',
        default_value="",
        description="Unique ID of this device",
    ))

    fcu_url = LaunchConfiguration('fcu_url')
    ld.add_action(DeclareLaunchArgument(
        'fcu_url',
        default_value="",
        description="URL address of the FCU to connect to",
    ))

    gcs_url = LaunchConfiguration('gcs_url')
    ld.add_action(DeclareLaunchArgument(
        'gcs_url',
        default_value="",
        description="URL address of the groundstation to connect to",
    ))

    tgt_component = LaunchConfiguration('tgt_component')
    ld.add_action(DeclareLaunchArgument(
        'tgt_component',
        default_value="1",
        description="This should always be 1, even when using something like a mavlink gimbal",
    ))

    fcu_protocol = LaunchConfiguration('fcu_protocol')
    ld.add_action(DeclareLaunchArgument(
        'fcu_protocol',
        default_value="v2.0",
        description="Version of mavlink protocol",
    ))

    respawn_mavros = LaunchConfiguration('respawn_mavros')
    ld.add_action(DeclareLaunchArgument(
        'respawn_mavros',
        default_value="false",
        description="Attempt to relaunch when mavros dies (might not work when using the mrs_drone_spawner)",
    ))

    log_output = LaunchConfiguration('log_output')
    ld.add_action(DeclareLaunchArgument(
        'log_output',
        default_value="screen",
        description="Logging destination (screen is console)",
    ))

    # #}

    tgt_system = PythonExpression(
            [ 'str(int(', LaunchConfiguration('ID'), ') + 1)']
    )

    uav_name = PythonExpression(
            [ '"uav" + str(', LaunchConfiguration('ID'), ')' ]
    )
    
    namespace = 'mavros'

    mavros_px4_config_file = LaunchConfiguration('generated_mavros_px4_config_file')

    ld.add_action(OpaqueFunction(
        function=lambda context: [
                DeclareLaunchArgument('generated_mavros_px4_config_file',
                default_value=generate_mavros_config(context)[0])
            ]
        )
    )

    mavros_launch_arguments = {
        "fcu_url": fcu_url,
        "gcs_url": gcs_url,
        "tgt_system": tgt_system,
        "tgt_component": tgt_component,
        "log_output": log_output,
        "fcu_protocol": fcu_protocol,
        "respawn_mavros": respawn_mavros,
        "namespace": namespace,
        "pluginlists_yaml":  this_pkg_path + "/config/mavros_plugins.yaml",
        # "config_yaml": this_pkg_path + "/config/mavros_px4_config.yaml"
        "config_yaml": mavros_px4_config_file
    }

    launch_xml_include_with_namespace = GroupAction(
        actions=[
            # push_ros_namespace first to set namespace of included nodes for following actions
            PushROSNamespace(uav_name),
            IncludeLaunchDescription(
                XMLLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('mrs_uav_px4_api'),
                        'launch/mavros.launch')
                    ),
                    launch_arguments=mavros_launch_arguments.items()
            ),
        ],
    )

    ld.add_action(launch_xml_include_with_namespace)

#     ld.add_action(
#         # Nodes under test
#         launch_ros.actions.Node(
#             package='tf2_ros',
#             namespace='',
#             executable='static_transform_publisher',
#             name='fcu_to_garmin',
#             arguments=["0.0", "0.0", "-0.05", "0", "1.57", "0", str(uav_name) + "/fcu", "garmin"],
#         )
#     )

    return ld


