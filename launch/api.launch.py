#!/usr/bin/env python3

import launch
import os
import re
import yaml

from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterFile
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration
from launch.substitutions import (
    LaunchConfiguration,
    IfElseSubstitution,
    PythonExpression,
    PathJoinSubstitution,
    EnvironmentVariable,
)

from ament_index_python.packages import get_package_share_directory

def mavros_settings(context, uav_name, this_pkg_path):
    '''MAVROS connection settings, "auto" args are derived from the run type and UAV_NAME'''

    simulation = context.perform_substitution(LaunchConfiguration('simulation')).lower() == 'true'
    old_px4_fw = os.getenv('OLD_PX4_FW', 'false') == 'true'
    px4_ip = os.getenv('PX4_IP', '')

    def arg_or_default(name, default):
        value = context.perform_substitution(LaunchConfiguration(name))
        return default if value == 'auto' else value

    if simulation:
        # the Gazebo spawner names vehicles uav<ID>, vehicle_base_port 14000, 4 ports per vehicle ID
        match = re.search(r'(\d+)$', uav_name)
        derived = [name for name in ['mavros_fcu_url', 'mavros_tgt_system'] if arg_or_default(name, None) is None]
        if not match and derived:
            # guessing an ID would silently connect to another vehicle's ports
            raise RuntimeError(f'cannot derive the vehicle ID from UAV_NAME "{uav_name}" (no trailing digits), set {" and ".join(derived)}')
        uav_id = int(match.group(1)) if match else 1
        local_port = 14000 + 4 * uav_id
        fcu_url = f'udp://127.0.0.1:{local_port + 1}@127.0.0.1:{local_port}'
        gcs_url = ''
        tgt_system = str(uav_id + 1)
    else:
        fcu_url = f'udp://:14550@{px4_ip}:14550' if px4_ip else '/dev/pixhawk:' + ('921600' if old_px4_fw else '2000000')
        gcs_url = 'tcp-l://'
        tgt_system = '1'

    # PX4 SITL reports the simulated Garmin as ROTATION_CUSTOM
    if simulation:
        garmin_id = '0'
        garmin_orientation = 'CUSTOM'
    else:
        garmin_id = '33' if old_px4_fw else '0'
        garmin_orientation = '0' if old_px4_fw else 'PITCH_270'

    return {
        'fcu_url': arg_or_default('mavros_fcu_url', fcu_url),
        'gcs_url': arg_or_default('mavros_gcs_url', gcs_url),
        'tgt_system': arg_or_default('mavros_tgt_system', tgt_system),
        'garmin_id': garmin_id,
        'garmin_orientation': garmin_orientation,
        'config_yaml': arg_or_default('mavros_config', this_pkg_path + '/config/mavros_px4_config.yaml'),
    }


def mavros_common(mavros, uav_name):
    '''Settings shared by the in-container and the standalone MAVROS'''

    tgt_system = int(mavros['tgt_system'])
    uas_url = f'/uas{tgt_system}'
    frame_namespace = uav_name + '/mavros'

    remappings = [
        ('/diagnostics', 'mavros/diagnostics'),
        (uas_url + '/mavlink_source', 'mavros/mavlink/source'),
        (uas_url + '/mavlink_sink', 'mavros/mavlink/sink'),
    ]

    frame_ids = [
        {'base_link_frame_id': frame_namespace + '/base_link'},
        {'odom_frame_id': frame_namespace + '/odom'},
        {'map_frame_id': frame_namespace + '/map'},
    ]

    # substitutions used in mavros_px4_config.yaml
    launch_configurations = [
        SetLaunchConfiguration('uav_name', uav_name),
        SetLaunchConfiguration('frame_namespace', frame_namespace),
        SetLaunchConfiguration('garmin_id', mavros['garmin_id']),
        SetLaunchConfiguration('garmin_orientation', mavros['garmin_orientation']),
    ]

    return tgt_system, uas_url, remappings, frame_ids, launch_configurations


def mavros_components(mavros, uav_name, use_sim_time, this_pkg_path):
    '''MAVROS router + UAS as components of hw_api_container (mavros >= 2.16)'''

    tgt_system, uas_url, remappings, frame_ids, launch_configurations = mavros_common(mavros, uav_name)

    with open(this_pkg_path + '/config/mavros_plugins.yaml') as f:
        plugin_allowlist = yaml.safe_load(f)['/**']['ros__parameters']['plugin_allowlist']

    router_params = [
        {'fcu_urls': [mavros['fcu_url']]},
        {'uas_urls': [uas_url]},
        {'use_sim_time': use_sim_time},
    ]

    if mavros['gcs_url']:
        router_params.append({'gcs_urls': [mavros['gcs_url']]})

    components = [

        ComposableNode(
            package='mavros',
            plugin='mavros::router::Router',
            namespace=uav_name,
            name='mavros_router',
            parameters=router_params,
            remappings=remappings,
            extra_arguments=[{'use_intra_process_comms': True}],
        ),

        # the UAS node is named "mavros", so the topics end up in /<uav_name>/mavros/...
        ComposableNode(
            package='mavros',
            plugin='mavros::uas::UAS',
            namespace=uav_name,
            name='mavros',
            parameters=[
                {'uas_url': uas_url},
                {'fcu_protocol': 'v2.0'},
                {'system_id': tgt_system},
                {'target_system_id': tgt_system},
                {'target_component_id': 1},
                {'plugin_allowlist': plugin_allowlist},
                *frame_ids,
                {'use_sim_time': use_sim_time},
            ],
            remappings=remappings,
            extra_arguments=[{'use_intra_process_comms': True}],
        ),
    ]

    return components, launch_configurations


def mavros_standalone(mavros, uav_name, use_sim_time, this_pkg_path):
    '''MAVROS as a separate mavros_node process'''

    tgt_system, _, remappings, frame_ids, launch_configurations = mavros_common(mavros, uav_name)

    # topics end up in /<uav_name>/mavros/..., the UAS node ("mavros") adds the mavros namespace
    # no name=, so the parameters are keyed by /** and reach the plugin nodes too
    node = Node(
        package='mavros',
        executable='mavros_node',
        namespace=uav_name,
        output='screen',
        respawn=os.getenv('respawn_mavros', 'false') == 'true',
        respawn_delay=2.0,
        additional_env={'MAVROS_EXECUTOR_TYPE': LaunchConfiguration('mavros_executor')},
        parameters=[
            {'fcu_url': mavros['fcu_url']},
            {'gcs_url': mavros['gcs_url']},
            {'tgt_system': tgt_system},
            {'tgt_component': 1},
            {'fcu_protocol': 'v2.0'},
            {'use_sim_time': use_sim_time},
            *frame_ids,
            ParameterFile(this_pkg_path + '/config/mavros_plugins.yaml', allow_substs=True),
            ParameterFile(mavros['config_yaml'], allow_substs=True),
        ],
        remappings=remappings,
    )

    return launch_configurations + [node]


def hw_api_container(context, uav_name, use_sim_time, configs, custom_config, platform_config, simulation, mavros_odometry_in_topic, this_pkg_path):

    in_container = context.perform_substitution(LaunchConfiguration('standalone')).lower() != 'true'
    simulation = context.perform_substitution(simulation).lower() == 'true'

    mavros = mavros_settings(context, uav_name, this_pkg_path)

    mavros_nodes = []
    container_options = {}

    if in_container:
        mavros_nodes, actions = mavros_components(mavros, uav_name, use_sim_time, this_pkg_path)
        container_options = {
            # mavros applies only the container's process-wide parameters to its plugin nodes
            'parameters': [ParameterFile(mavros['config_yaml'], allow_substs=True)],
            'ros_arguments': ['-p', 'use_sim_time:=' + str(use_sim_time).lower()],
            'additional_env': {'MAVROS_EXECUTOR_TYPE': LaunchConfiguration('mavros_executor')},
        }
    else:
        actions = mavros_standalone(mavros, uav_name, use_sim_time, this_pkg_path)

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
    ld.add_action(DeclareLaunchArgument('mavros_config', default_value='auto', description='MAVROS plugin config yaml, "auto" uses config/mavros_px4_config.yaml'))

    # #} end of mavros

    ld.add_action(OpaqueFunction(
        function=hw_api_container,
        args=[uav_name, use_sim_time, configs, custom_config, platform_config, simulation, mavros_odometry_in_topic, this_pkg_path],
    ))

    return ld
