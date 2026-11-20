"""Gazebo spawner status for launch/api.launch.py: wait for the own vehicle, take its platform config and MAVLink connection.

rclpy and the spawner messages are imported only where needed, a real-world launch does not need them.
"""

import atexit
import contextlib
import os
import tempfile
import time

import launch

SPAWNER_STATUS_TOPIC = '/mrs_drone_spawner/status'


# #{ find_vehicle(status, uav_name)
def find_vehicle(status, uav_name):
    '''The status entry of the vehicle, None if the spawner does not know it'''
    return next((vehicle for vehicle in status.vehicles if vehicle.name == uav_name), None)
# #}


# #{ wait_for_spawned_vehicle(uav_name, log_period, log)
def wait_for_spawned_vehicle(uav_name, log_period=5.0, log=None):
    '''Blocks until the Gazebo spawner reports the vehicle spawned, returns its GazeboSpawnerVehicle.

    It blocks the launch's event loop: do not include this launch file into the launch that starts the spawner.
    Ctrl-C raises KeyboardInterrupt here (default handler), launch then shuts down.'''
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from mrs_msgs.msg import GazeboSpawnerStatus, GazeboSpawnerVehicle

    log = log or launch.logging.get_logger('api.launch.py').info

    context = rclpy.Context()
    # args=[]: do not parse the launch's command line
    rclpy.init(args=[], context=context)

    try:
        node = rclpy.create_node('api_launch_spawner_status', namespace=uav_name, context=context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)

        try:
            status = None

            def callback_status(msg):
                nonlocal status
                status = msg

            latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            node.create_subscription(GazeboSpawnerStatus, SPAWNER_STATUS_TOPIC, callback_status, latched)

            last_log = None

            while True:
                executor.spin_once(timeout_sec=0.1)

                vehicle = find_vehicle(status, uav_name) if status else None

                if vehicle is not None and vehicle.state == GazeboSpawnerVehicle.STATE_SPAWNED and vehicle.platform_config:
                    return vehicle

                now = time.monotonic()
                if last_log is None or now - last_log >= log_period:
                    message = f'waiting for {SPAWNER_STATUS_TOPIC} for {uav_name}'
                    if vehicle is not None and vehicle.state == GazeboSpawnerVehicle.STATE_FAILED:
                        message += f' (spawn failed: {vehicle.message})'
                    log(message)
                    last_log = now
        finally:
            executor.shutdown()
            node.destroy_node()
    finally:
        rclpy.shutdown(context=context)
# #}


# #{ spawner_configs(vehicle, configs)
def spawner_configs(vehicle, configs):
    '''Writes the vehicle's platform config from the spawner to a file and puts it before the package configs'''
    with tempfile.NamedTemporaryFile('w', prefix=f'hw_api_spawner_{vehicle.name}_', suffix='.yaml', delete=False) as f:
        f.write(vehicle.platform_config)

    # the launch process outlives the HW API manager, which reads the file at its start
    atexit.register(remove_file, f.name)

    return [f.name] + configs
# #}


# #{ remove_file(path)
def remove_file(path):
    with contextlib.suppress(FileNotFoundError):
        os.remove(path)
# #}
