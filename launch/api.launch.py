#!/usr/bin/env python3

import launch
import os

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterFile
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import (
    LaunchConfiguration,
    IfElseSubstitution,
    PythonExpression,
    PathJoinSubstitution,
    EnvironmentVariable,
)

from ament_index_python.packages import get_package_share_directory

from mrs_uav_px4_api.mavros import mavros_config_path, mavros_settings, mavros_components, mavros_standalone, mavros_namespace_rules


# #{ hw_api_container(context, uav_name, use_sim_time, old_px4_fw, configs, custom_config, platform_config, simulation, mavros_odometry_in_topic, this_pkg_path)
def hw_api_container(context, uav_name, use_sim_time, old_px4_fw, configs, custom_config, platform_config, simulation, mavros_odometry_in_topic, this_pkg_path):

    in_container = context.perform_substitution(LaunchConfiguration('standalone')).lower() != 'true'
    simulation = context.perform_substitution(simulation).lower() == 'true'

    mavros = mavros_settings(context, simulation, old_px4_fw, uav_name, mavros_config_path(context), this_pkg_path)

    container_options = {}

    if in_container:
        mavros_nodes, actions = mavros_components(mavros, uav_name, use_sim_time)
        container_options = {
            # mavros applies only the container's process-wide parameters to its plugin nodes,
            # they don't need mavros_plugins.yaml, the UAS loads it as a component
            'parameters': [ParameterFile(path, allow_substs=True) for path in mavros['config_files']],
            # the MAVROS plugin nodes go to /<uav_name>/mavros, the container stays in /<uav_name>
            # (the components set their own namespace)
            'ros_arguments': mavros_namespace_rules(uav_name, ['hw_api_container']) + ['-p', 'use_sim_time:=' + str(use_sim_time).lower()],
            'additional_env': {'MAVROS_EXECUTOR_TYPE': LaunchConfiguration('mavros_executor')},
        }
    else:
        mavros_nodes, actions = mavros_standalone(mavros, uav_name, use_sim_time)

    hw_api_node = ComposableNode(
        package='mrs_uav_hw_api',
        plugin='mrs_uav_hw_api::HwApiManager',
        namespace=uav_name,
        name='hw_api',
        parameters=[
            {'uav_name': uav_name},
            {'topic_prefix': '/' + uav_name},
            {'use_sim_time': use_sim_time},
            {'configs': configs},
            {'custom_config': custom_config},
            {'platform_config': platform_config},
            {'simulation': simulation},
        ],

        remappings=[
            ('~/ground_truth_in', 'ground_truth' if simulation else 'rtk/bestpos'),
            ('~/mavros_state_in', 'mavros/state'),
            ('~/mavros_extended_state_in', 'mavros/extended_state'),
            ('~/mavros_local_position_in', 'mavros/local_position/odom'),
            ('~/mavros_odometry_in', mavros_odometry_in_topic),
            ('~/mavros_global_position_in', 'mavros/global_position/global'),
            ('~/mavros_garmin_in', 'mavros/garmin'),
            ('~/mavros_imu_in', 'mavros/imu/data'),
            ('~/mavros_magnetometer_in', 'mavros/global_position/compass_hdg'),
            ('~/mavros_magnetic_field_in', 'mavros/imu/mag'),
            ('~/mavros_rc_in', 'mavros/rc/in'),
            ('~/mavros_altitude_in', 'mavros/altitude'),
            ('~/mavros_battery_in', 'mavros/battery'),
            ('~/mavros_gps_status_raw_in', 'mavros/gpsstatus/gps1/raw'),

            ('~/mavros_cmd_out', 'mavros/cmd/command'),
            ('~/mavros_set_mode_out', 'mavros/set_mode'),
            ('~/mavros_attitude_setpoint_out', 'mavros/setpoint_raw/attitude'),
            ('~/mavros_actuator_control_out', 'mavros/actuator_control'),
            ('~/errors', 'errors'),
        ],

        # mavros -> hw_api messages are passed in-process when mavros runs in this container
        extra_arguments=[{'use_intra_process_comms': in_container}],
    )

    actions.append(ComposableNodeContainer(
        namespace=uav_name,
        name='hw_api_container',
        package='rclcpp_components',
        executable='component_container_events_cbg',
        output='screen',
        **container_options,
        composable_node_descriptions=mavros_nodes + [hw_api_node],
    ))

    return actions
# #}


def generate_launch_description():

    ld = launch.LaunchDescription()

    pkg_name = 'mrs_uav_px4_api'

    this_pkg_path = get_package_share_directory(pkg_name)

    # #{ custom_config

    custom_config = LaunchConfiguration('custom_config')

    # this adds the args to the list of args available for this launch files
    # these args can be listed at runtime using -s flag
    # default_value is required to if the arg is supposed to be optional at launch time
    ld.add_action(DeclareLaunchArgument(
        'custom_config',
        default_value='',
        description='Path to the custom configuration file. The path can be absolute, starting with "/" or relative to the current working directory',
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

    # #{ platform_config

    platform_config = LaunchConfiguration('platform_config')

    ld.add_action(DeclareLaunchArgument(
        'platform_config',
        default_value='',
        description='Path to the platform configuration file. The path can be absolute, starting with "/" or relative to the current working directory',
    ))

    # behaviour:
    #     platform_config == "" => platform_config: ""
    #     platform_config == "/<path>" => platform_config: "/<path>"
    #     platform_config == "<path>" => platform_config: "$(pwd)/<path>"
    platform_config = IfElseSubstitution(
            condition=PythonExpression(['"', platform_config, '" != "" and ', 'not "', platform_config, '".startswith("/")']),
            if_value=PathJoinSubstitution([EnvironmentVariable('PWD'), platform_config]),
            else_value=platform_config
    )

    # #} end of platform_config

    # #{ simulation

    simulation = LaunchConfiguration('simulation')

    declare_simulation = DeclareLaunchArgument(
        'simulation',
        default_value='true' if os.getenv('RUN_TYPE', 'realworld') == 'simulation' else 'false',
        description='Whether to run in simulation mode.'
    )

    ld.add_action(declare_simulation)

    # #} end of simulation

    # #{ args from ENV

    uav_name = os.getenv('UAV_NAME', 'uav1')
    use_sim_time = os.getenv('USE_SIM_TIME', 'false') == 'true'
    old_px4_fw = os.getenv('OLD_PX4_FW', 'false') == 'true'

    # #} end of args from ENV

    if old_px4_fw:
        mavros_odometry_in_topic = 'mavros/local_position/odom'
    else:
        mavros_odometry_in_topic = 'mavros/odometry/in'

    # the first one has the priority
    configs = [
        this_pkg_path + '/config/px4_api.yaml',
        get_package_share_directory('mrs_uav_hw_api') + '/config/hw_api.yaml',
    ]

    # #{ mavros

    ld.add_action(DeclareLaunchArgument(
        'standalone',
        default_value='false',
        description='Whether to start MAVROS as a standalone process or load it into hw_api_container (mavros >= 2.16).',
    ))

    ld.add_action(DeclareLaunchArgument(
        'mavros_executor',
        default_value='events',
        description='MAVROS executor type (MAVROS_EXECUTOR_TYPE): events or mt',
    ))

    # "auto" derives the value from the run type and UAV_NAME
    ld.add_action(DeclareLaunchArgument('mavros_fcu_url', default_value='auto', description='FCU URL used by MAVROS'))
    ld.add_action(DeclareLaunchArgument('mavros_gcs_url', default_value='auto', description='GCS URL used by MAVROS'))
    ld.add_action(DeclareLaunchArgument('mavros_tgt_system', default_value='auto', description='Target system ID for MAVROS'))
    ld.add_action(DeclareLaunchArgument(
        'mavros_config',
        default_value='',
        description='Custom MAVROS config yaml loaded on top of config/mavros_px4_config.yaml and config/mavros_plugins.yaml (incl. plugin_allowlist). '
                    'The path can be absolute or relative to the current working directory',
    ))

    # #} end of mavros

    ld.add_action(OpaqueFunction(
        function=hw_api_container,
        args=[uav_name, use_sim_time, old_px4_fw, configs, custom_config, platform_config, simulation, mavros_odometry_in_topic, this_pkg_path],
    ))

    return ld
