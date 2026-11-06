#!/usr/bin/env python3
# Tests of the Gazebo spawner status helpers (mrs_uav_px4_api.gazebo_spawner, mrs_uav_px4_api.mavros), run manually:
#   python3 -m pytest -q test/test_spawner_status_launch.py
# (sourced workspace; the wait tests need a running ROS middleware, e.g. rmw_zenohd)

import threading
import time

import pytest
import rclpy
import yaml
from launch import LaunchContext
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from mrs_msgs.msg import GazeboSpawnerStatus, GazeboSpawnerVehicle

from mrs_uav_px4_api.gazebo_spawner import find_vehicle, spawner_configs, wait_for_spawned_vehicle
from mrs_uav_px4_api.mavros import mavros_settings

CFG = 'mrs_uav_hw_api:\n  outputs:\n    distance_sensor: true\n    ground_truth: false\n'


def vehicle(name, state=GazeboSpawnerVehicle.STATE_SPAWNED, url='udp://127.0.0.1:15005@127.0.0.1:15004', system_id=2, message=''):
    return GazeboSpawnerVehicle(name=name,
                                state=state,
                                message=message,
                                platform_config=CFG if state == GazeboSpawnerVehicle.STATE_SPAWNED else '',
                                mavlink_fcu_url=url,
                                mavlink_system_id=system_id)


def context(**args):
    ctx = LaunchContext()
    defaults = {'simulation': 'true', 'mavros_fcu_url': 'auto', 'mavros_gcs_url': 'auto', 'mavros_tgt_system': 'auto', 'mavros_config': ''}
    defaults.update(args)
    for key, value in defaults.items():
        ctx.launch_configurations[key] = value
    return ctx


# #{ lookup and settings
def test_vehicle_lookup_exact_name():
    status = GazeboSpawnerStatus(vehicles=[vehicle('uav10'), vehicle('uav1x')])
    assert find_vehicle(status, 'uav1') is None
    status.vehicles.append(vehicle('uav1'))
    assert find_vehicle(status, 'uav1').name == 'uav1'


def test_mavros_settings_from_vehicle():
    settings = mavros_settings(context(), True, False, vehicle('uav1'), '', '/pkg')
    assert settings['fcu_url'] == 'udp://127.0.0.1:15005@127.0.0.1:15004'
    assert settings['tgt_system'] == 2

    settings = mavros_settings(context(mavros_fcu_url='udp://:1@2.3.4.5:6'), True, False, vehicle('uav1'), '', '/pkg')
    assert settings['fcu_url'] == 'udp://:1@2.3.4.5:6'


def test_mavros_settings_without_vehicle():
    # spawner_status:=false: the mavros_* args are required
    with pytest.raises(RuntimeError, match='set mavros_fcu_url and mavros_tgt_system'):
        mavros_settings(context(), True, False, None, '', '/pkg')

    settings = mavros_settings(context(mavros_fcu_url='udp://:1@2.3.4.5:6', mavros_tgt_system='3'), True, False, None, '', '/pkg')
    assert settings['fcu_url'] == 'udp://:1@2.3.4.5:6'
    assert settings['tgt_system'] == 3


def test_written_config_is_first():
    configs = spawner_configs(vehicle('uav1'), ['/pkg/config/px4_api.yaml', '/hw_api/config/hw_api.yaml'])
    assert configs[1:] == ['/pkg/config/px4_api.yaml', '/hw_api/config/hw_api.yaml']
    with open(configs[0]) as f:
        assert yaml.safe_load(f) == yaml.safe_load(CFG)


# #} end of lookup and settings


# #{ waiting for the spawner
@pytest.fixture
def status_publisher():
    '''Publishes a latched GazeboSpawnerStatus from its own context, like the spawner'''
    ctx = rclpy.Context()
    rclpy.init(context=ctx)
    node = rclpy.create_node('fake_mrs_drone_spawner', namespace='/mrs_drone_spawner', context=ctx)
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    pub = node.create_publisher(GazeboSpawnerStatus, '/mrs_drone_spawner/status', qos)
    yield pub
    node.destroy_node()
    rclpy.shutdown(context=ctx)


@pytest.fixture
def publish_later(status_publisher):
    timers = []

    def schedule(delay, vehicles):
        timers.append(threading.Timer(delay, lambda: status_publisher.publish(GazeboSpawnerStatus(vehicles=vehicles))))
        timers[-1].start()

    yield schedule

    for timer in timers:
        timer.cancel()
        timer.join()


def wait(uav_name, lines, timeout=10.0):
    '''wait_for_spawned_vehicle in a thread, so that a regression fails instead of hanging the tests'''
    result = []
    thread = threading.Thread(target=lambda: result.append(wait_for_spawned_vehicle(uav_name, log_period=0.2, log=lines.append)),
                              daemon=True)
    thread.start()
    thread.join(timeout)
    assert result, f'not spawned within {timeout} s'
    return result[0]


def test_failed_keeps_waiting(status_publisher, publish_later):
    lines = []
    status_publisher.publish(GazeboSpawnerStatus(vehicles=[vehicle('uav1', GazeboSpawnerVehicle.STATE_FAILED, message='boom')]))
    publish_later(1.0, [vehicle('uav1')])

    result = wait('uav1', lines)

    assert result.state == GazeboSpawnerVehicle.STATE_SPAWNED
    assert result.platform_config == CFG
    assert any('boom' in line for line in lines)


def test_wait_logs_and_continues(publish_later):
    lines = []
    publish_later(1.0, [vehicle('uav1')])

    start = time.time()
    result = wait('uav1', lines)

    assert result.name == 'uav1'
    assert time.time() - start >= 0.9
    assert any('waiting for /mrs_drone_spawner/status for uav1' in line for line in lines)


# #} end of waiting for the spawner
