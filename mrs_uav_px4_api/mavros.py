"""MAVROS settings and nodes for launch/api.launch.py"""

import os
import re

from launch.actions import SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterFile


# #{ mavros_config_path(context)
def mavros_config_path(context):
    '''The custom MAVROS config, loaded on top of the package defaults; a relative path is relative to the current working directory'''

    custom_config = context.perform_substitution(LaunchConfiguration('mavros_config'))

    if custom_config:
        custom_config = os.path.join(os.getenv('PWD', os.getcwd()), custom_config)
        if not os.path.isfile(custom_config):
            raise RuntimeError(f'mavros_config "{custom_config}" does not exist')

    return custom_config


# #}


# #{ mavros_settings(context, simulation, old_px4_fw, uav_name, custom_config, this_pkg_path)
def mavros_settings(context, simulation, old_px4_fw, uav_name, custom_config, this_pkg_path):
    '''MAVROS connection and config files: the connection follows from the run type and UAV_NAME, the mavros_* launch args override it'''

    if simulation:
        # the Gazebo spawner names vehicles uav<ID>, each uses the UDP ports from 14000 + 4 * ID and MAV_SYS_ID = ID + 1
        fcu_url = None
        tgt_system = None
        match = re.search(r'(\d+)$', uav_name)
        if match:
            uav_id = int(match.group(1))
            port = 14000 + 4 * uav_id
            fcu_url = f'udp://127.0.0.1:{port + 1}@127.0.0.1:{port}'
            tgt_system = str(uav_id + 1)
        gcs_url = ''
        # PX4 SITL reports the simulated Garmin as ROTATION_CUSTOM
        garmin_id = '0'
        garmin_orientation = 'CUSTOM'
    else:
        px4_ip = os.getenv('PX4_IP', '')
        if px4_ip:
            fcu_url = f'udp://:14550@{px4_ip}:14550'
        elif old_px4_fw:
            fcu_url = '/dev/pixhawk:921600'
        else:
            fcu_url = '/dev/pixhawk:2000000'
        gcs_url = 'tcp-l://'
        tgt_system = '1'
        if old_px4_fw:
            garmin_id = '33'
            garmin_orientation = '0'
        else:
            garmin_id = '0'
            garmin_orientation = 'PITCH_270'

    def arg_or_default(name, default):
        value = context.perform_substitution(LaunchConfiguration(name))
        return default if value == 'auto' else value

    fcu_url = arg_or_default('mavros_fcu_url', fcu_url)
    gcs_url = arg_or_default('mavros_gcs_url', gcs_url)
    tgt_system = arg_or_default('mavros_tgt_system', tgt_system)

    missing = [name for name, value in [('mavros_fcu_url', fcu_url), ('mavros_tgt_system', tgt_system)] if value is None]
    if missing:
        raise RuntimeError(f'cannot derive the vehicle ID from UAV_NAME "{uav_name}" (no trailing digits), set {" and ".join(missing)}')

    config_files = [this_pkg_path + '/config/mavros_px4_config.yaml']
    if custom_config:
        config_files.append(custom_config)

    return {
        'fcu_url': fcu_url,
        'gcs_url': gcs_url,
        'tgt_system': int(tgt_system),
        'garmin_id': garmin_id,
        'garmin_orientation': garmin_orientation,
        'plugins_file': this_pkg_path + '/config/mavros_plugins.yaml',
        'config_files': config_files,
    }


# #}


# #{ mavros_remappings(mavros)
def mavros_remappings(mavros):
    '''Per-vehicle MAVLink and diagnostics topics, so that several vehicles do not share them'''

    uas_url = f'/uas{mavros["tgt_system"]}'

    return [
        ('/diagnostics', 'mavros/diagnostics'),
        (uas_url + '/mavlink_source', 'mavros/mavlink/source'),
        (uas_url + '/mavlink_sink', 'mavros/mavlink/sink'),
    ]


# #}


# #{ mavros_frame_ids(uav_name)
def mavros_frame_ids(uav_name):
    '''TF frames of the vehicle, in the <uav_name>/mavros namespace'''

    return [
        {'base_link_frame_id': uav_name + '/mavros/base_link'},
        {'odom_frame_id': uav_name + '/mavros/odom'},
        {'map_frame_id': uav_name + '/mavros/map'},
    ]


# #}


# #{ mavros_launch_configurations(mavros, uav_name)
def mavros_launch_configurations(mavros, uav_name):
    '''Substitutions used in mavros_px4_config.yaml'''

    return [
        SetLaunchConfiguration('uav_name', uav_name),
        SetLaunchConfiguration('frame_namespace', uav_name + '/mavros'),
        SetLaunchConfiguration('garmin_id', mavros['garmin_id']),
        SetLaunchConfiguration('garmin_orientation', mavros['garmin_orientation']),
    ]


# #}


# #{ mavros_components(mavros, uav_name, use_sim_time)
def mavros_components(mavros, uav_name, use_sim_time):
    '''MAVROS router + UAS as components of hw_api_container (mavros >= 2.16); returns (components, launch actions)'''

    tgt_system = mavros['tgt_system']
    uas_url = f'/uas{tgt_system}'

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
            remappings=mavros_remappings(mavros),
            extra_arguments=[{'use_intra_process_comms': True}],
        ),

        # the UAS node is named "mavros", so the topics end up in /<uav_name>/mavros/...
        ComposableNode(
            package='mavros',
            plugin='mavros::uas::UAS',
            namespace=uav_name,
            name='mavros',
            # components don't get the container's parameter files (use_global_arguments(false)), so the UAS loads
            # mavros_plugins.yaml (plugin_allowlist) and the config files itself,
            # the explicit parameters below go last so the files can't override them
            parameters=[
                ParameterFile(mavros['plugins_file'], allow_substs=True),
                *[ParameterFile(path, allow_substs=True) for path in mavros['config_files']],
                {'uas_url': uas_url},
                {'system_id': tgt_system},
                {'target_system_id': tgt_system},
                *mavros_frame_ids(uav_name),
                {'use_sim_time': use_sim_time},
            ],
            remappings=mavros_remappings(mavros),
            extra_arguments=[{'use_intra_process_comms': True}],
        ),
    ]

    return components, mavros_launch_configurations(mavros, uav_name)


# #}


# #{ mavros_standalone(mavros, uav_name, use_sim_time)
def mavros_standalone(mavros, uav_name, use_sim_time):
    '''MAVROS as a separate mavros_node process; returns (components, launch actions) like mavros_components, with no components'''

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
        # the explicit parameters go last so the files can't override them (same as in the container)
        parameters=[
            ParameterFile(mavros['plugins_file'], allow_substs=True),
            *[ParameterFile(path, allow_substs=True) for path in mavros['config_files']],
            {'fcu_url': mavros['fcu_url']},
            {'gcs_url': mavros['gcs_url']},
            {'tgt_system': mavros['tgt_system']},
            {'use_sim_time': use_sim_time},
            *mavros_frame_ids(uav_name),
        ],
        remappings=mavros_remappings(mavros),
    )

    return [], mavros_launch_configurations(mavros, uav_name) + [node]


# #}
