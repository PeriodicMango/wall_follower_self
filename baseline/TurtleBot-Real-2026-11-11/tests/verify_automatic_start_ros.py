"""Exercise the real automatic-start worker with Humble AMCL and a synthetic LDS on a private loopback DDS graph.

Run with scripts/real_env.sh python3 /absolute/path/to/this_file. No motor publishers or action clients are created.
"""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import uuid

BASE = Path(__file__).resolve().parents[1]
RUN = BASE / "runs/Real_103_2026-11-11_152732"
MAP = RUN / "maps/maze_Real_103_2026-11-11_152732.yaml"
START = RUN / "maps/markers_Real_103_2026-11-11_152732.start_map.txt"


def loopback(directory):
    # The real worker checks Domain 3. Bind the only interface to localhost and
    # assign a random discovery tag on both clients and servers in this test.
    tag = "placement_test_" + uuid.uuid4().hex
    os.environ.update(ROS_DOMAIN_ID="3", ROS_LOCALHOST_ONLY="0", RMW_IMPLEMENTATION="rmw_cyclonedds_cpp",
                      ROS_LOG_DIR=str(directory), CYCLONEDDS_URI=(
                          '<CycloneDDS><Domain Id="3"><General><Interfaces><NetworkInterface address="127.0.0.1"/>'
                          '</Interfaces><AllowMulticast>false</AllowMulticast></General><Discovery>'
                          f'<Tag>{tag}</Tag><ParticipantIndex>auto</ParticipantIndex><Peers>'
                          '<Peer Address="127.0.0.1"/></Peers></Discovery></Domain></CycloneDDS>'))


def render_scan(grid, resolution, origin, pose):
    """Intersections sampled every 8 mm; unobserved cells end a ray with no return."""
    import numpy as np
    x, y, yaw = pose
    ox, oy, offset_yaw = origin
    assert abs(offset_yaw) < 1e-10
    scan_x = x - .064 * math.cos(yaw)
    scan_y = y - .064 * math.sin(yaw)
    angles = np.deg2rad(np.arange(360)) + yaw
    ray = np.arange(.008, 3.51, .008)
    gx = np.floor((scan_x - ox + np.cos(angles[:, None]) * ray) / resolution).astype(int)
    gy = np.floor((scan_y - oy + np.sin(angles[:, None]) * ray) / resolution).astype(int)
    valid = (gx >= 0) & (gx < grid.shape[1]) & (gy >= 0) & (gy < grid.shape[0])
    pixel = np.full(gx.shape, 205, dtype=np.uint8)
    pixel[valid] = grid[gy[valid], gx[valid]]
    boundary = (pixel < 220)
    hit_i = boundary.argmax(axis=1)
    occupied = boundary.any(axis=1) & (pixel[np.arange(360), hit_i] < 64)
    distance = np.where(occupied & (ray[hit_i] >= .12), ray[hit_i], np.inf)
    return distance


def run(folder):
    import rclpy
    import yaml
    import numpy as np
    from PIL import Image
    from ament_index_python.packages import get_package_prefix
    from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
    from lifecycle_msgs.srv import ChangeState
    from nav_msgs.msg import Odometry
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan
    from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

    meta = yaml.safe_load(MAP.read_text())
    grid = np.flipud(np.array(Image.open(MAP.parent / meta['image'])))
    saved = tuple(map(float, START.read_text().strip().split(',')))
    # Occlude/perturb ideal map geometry instead of feeding the same scan on every tick.
    rng = np.random.default_rng(62513)
    poses = {'start': saved, 'wrong': (1.5, -1.25, 1.4), 'reverse': (saved[0], saved[1], saved[2] + math.pi)}
    cache = {key: render_scan(grid, meta['resolution'], meta['origin'], pose) for key, pose in poses.items()}
    phase = 'wrong'
    moving = False
    rclpy.init()
    odom = rclpy.create_node('diff_drive_controller')
    lidar = rclpy.create_node('hlds_laser_publisher')
    urdf = rclpy.create_node('robot_state_publisher')
    monitor = rclpy.create_node('placement_test_harness')
    executor = SingleThreadedExecutor()
    for n in (odom, lidar, urdf, monitor): executor.add_node(n)
    p_odom = odom.create_publisher(Odometry, '/odom', qos_profile_sensor_data)
    p_scan = lidar.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
    odom_tf = TransformBroadcaster(odom)
    base_tf = StaticTransformBroadcaster(urdf)
    seeds, amcl_estimates = [], []
    monitor.create_subscription(PoseWithCovarianceStamped,'/initialpose',seeds.append,10)
    monitor.create_subscription(PoseWithCovarianceStamped,'/amcl_pose',amcl_estimates.append,10)

    def tf(parent, child, stamp, x=0., z=0.):
        t = TransformStamped()
        t.header.frame_id = parent
        t.child_frame_id = child
        t.header.stamp = stamp
        t.transform.translation.x = x
        t.transform.translation.z = z
        t.transform.rotation.w = 1.
        return t

    base_tf.sendTransform(tf('base_footprint', 'base_scan', urdf.get_clock().now().to_msg(), -.064, .15))
    last_scan = last_odom = 0.
    def tick():
        nonlocal last_scan, last_odom
        now = time.monotonic()
        stamp = odom.get_clock().now().to_msg()
        if now - last_odom >= .027:
            m = Odometry()
            m.header.frame_id = 'odom'
            m.child_frame_id = 'base_footprint'
            m.header.stamp = stamp
            m.pose.pose.orientation.w = 1.
            m.twist.twist.linear.x = .12 if moving else 0.
            p_odom.publish(m)
            odom_tf.sendTransform(tf('odom', 'base_footprint', stamp))
            last_odom = now
        if now - last_scan >= .145:
            s = LaserScan()
            s.header.frame_id = 'base_scan'
            s.header.stamp = stamp
            s.angle_min = 0.
            s.angle_increment = math.tau / 360
            s.angle_max = s.angle_increment * 359
            s.range_min, s.range_max = .12, 3.5
            s.scan_time = .145
            measures = cache[phase] + rng.normal(0., .003, 360)
            measures[::89] = np.inf
            s.ranges = measures.astype('float32').tolist()
            p_scan.publish(s)
            last_scan = now
        executor.spin_once(timeout_sec=.008)

    def set_phase(value):
        nonlocal phase, moving
        if value == 'wheels_moving':
            moving = True
        else:
            phase, moving = value, False

    procs = []
    def spawn(name, command):
        log = (folder / (name+'.log')).open('w')
        try:
            proc = subprocess.Popen(command, stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT,
                                    start_new_session=True)
        finally: log.close()
        procs.append((name, proc))
        return proc

    def until(name, fn, timeout=16):
        end = time.monotonic()+timeout
        while not fn():
            for p_name, p in procs:
                if p.poll() is not None:
                    raise RuntimeError(f'{p_name} exited {p.returncode}: '+str((folder/(p_name+'.log')).read_text()[-1400:]))
            if time.monotonic() >= end:
                raise TimeoutError(f'{name}; see {folder}')
            tick()

    def sit(seconds, check=None):
        end = time.monotonic()+seconds
        while time.monotonic()<end:
            if check: check()
            tick()

    def lifecycle(name):
        svc = monitor.create_client(ChangeState, '/'+name+'/change_state')
        until(name+' service', svc.service_is_ready)
        for action in (1,3):
            req=ChangeState.Request()
            req.transition.id=action
            answer=svc.call_async(req)
            until(name+' transition', answer.done)
            assert answer.result().success, (name, action)

    params = yaml.safe_load((BASE / 'config/nav2_robot103.yaml').read_text())
    params['map_server']['ros__parameters']['yaml_filename'] = str(MAP)
    config = folder/'params.yaml'
    config.write_text(yaml.safe_dump({k:params[k] for k in ('amcl','map_server')}))
    try:
        for package, executable in [('nav2_map_server','map_server'),('nav2_amcl','amcl')]:
            program = Path(get_package_prefix(package)) / 'lib' / package / executable
            spawn(executable,[str(program),'--ros-args','--params-file',str(config)])
            lifecycle(executable)
        exercise_worker(folder, spawn, until, sit, lambda: len(seeds), lambda: len(amcl_estimates),
                        set_phase, saved)
    finally:
        for _, p in reversed(procs):
            if p.poll() is None: os.killpg(p.pid,signal.SIGINT)
        for _, p in reversed(procs):
            try: p.wait(3)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid,signal.SIGKILL); p.wait()
        for n in (monitor, urdf, lidar, odom): n.destroy_node()
        executor.shutdown()
        rclpy.try_shutdown()


def exercise_worker(folder, spawn, until, sit, seeds, estimates, set_phase, saved):
    print("Map server and actual Humble AMCL active", flush=True)
    if '--smoke' in sys.argv:
        sit(.5)
        print("ROS sensors verified", flush=True)
        return
    run_placement_protocol(folder, spawn, until, sit, seeds, estimates, set_phase, saved)


def read_payload(path):
    if not path.exists():
        return {}
    return json.loads(path.read_text())


def assert_no_token(path):
    assert read_payload(path).get('arm_token') is None


def atomic_write(path, payload):
    staged = path.with_suffix('.tmp')
    staged.write_text(json.dumps(payload))
    staged.replace(path)


def run_placement_protocol(folder, spawn, until, sit, seeds, estimates, set_phase, saved):
    measured = folder/'measured_pose.json'
    request = folder/'countdown-request.json'
    ready = folder/'ready.json'
    release = folder/'release.json'
    started = time.monotonic()
    events = []

    def trace(event, **fields):
        events.append(dict(at=round(time.monotonic()-started, 2), event=event, **fields))
        print(events[-1], flush=True)
        (folder/'events.json').write_text(json.dumps(events, indent=2))

    def no_ready():
        assert not read_payload(ready).get('ready'), f'unexpected clearance: {read_payload(ready)}'

    def generation():
        return read_payload(measured).get('generation')

    def wait_pose(phase, previous=None):
        set_phase(phase)
        until(phase+' new measured pose', lambda: bool(generation()) and generation() != previous, 22)
        pose = read_payload(measured)
        px, py, yaw = pose['map_pose']
        expected_yaw = saved[2] + (math.pi if phase == 'reverse' else 0)
        assert math.hypot(px-saved[0], py-saved[1]) < .15, pose
        assert abs(math.remainder(yaw-expected_yaw, math.tau)) < math.radians(16), pose
        trace('validated pose', phase=phase, generation=pose['generation'],
              amcl=[round(px, 3), round(py, 3), round(yaw, 3)])
        return pose['generation']

    proc = spawn('worker', [sys.executable, '-Bu', str(BASE/'scripts/auto_start_localization.py'),
                           '--map', str(MAP), '--home-file', str(START), '--output', str(measured),
                           '--watch-status', str(ready), '--countdown-request', str(request),
                           '--arm-request', str(release), '--timeout', '100'])
    sit(4, no_ready)
    assert not measured.exists() and seeds()==0, 'wrong-corner pose should not seed AMCL'
    trace('wrong corner withheld seed')
    first = wait_pose('start')
    assert seeds() > 0 and estimates() > 0
    sit(3.4, no_ready)
    trace('without goal preparation no clearance')
    atomic_write(request, {'generation':first})
    sit(1.4, no_ready)
    set_phase('wrong')  # carried without a wheel-odometry change
    sit(3.5, no_ready)
    trace('lift into different corridor revoked first countdown')
    second = wait_pose('reverse', first)
    assert second != first
    sit(3.4, no_ready)  # request from former generation cannot authorize this one
    trace('stale countdown generation ignored')
    atomic_write(request, {'generation':second})
    sit(1.2, no_ready)
    set_phase('wheels_moving')  # disrupt on same location; twist alone is enough here
    sit(3.3, no_ready)
    trace('wheel motion revoked second countdown')
    third = wait_pose('start', second)
    atomic_write(request, {'generation':third})
    mark = time.monotonic()
    observed_seconds = set()
    def count_complete():
        status = read_payload(ready)
        if status.get('phase') == 'countdown':
            for value in (1, 2, 3):
                if f'{value} 秒' in status.get('reason', ''):
                    observed_seconds.add(value)
        return status.get('ready') is True
    until('three-second verified countdown', count_complete, 10)
    assert observed_seconds == {1, 2, 3}, observed_seconds
    delta = time.monotonic() - mark
    assert delta >= 2.95, f'premature grant {delta:.2f}'
    grant = read_payload(ready)
    assert grant['generation'] == third, grant
    trace('full countdown', elapsed=round(delta,3), grant=grant)
    assert grant['arm_token'] is None
    atomic_write(release, {'generation':first,'token':'stale-generation-token'})
    sit(.8, lambda: assert_no_token(ready))
    atomic_write(release, {'generation':third,'token':'test-arm-ack'})
    until('fresh scan acknowledges token', lambda: read_payload(ready).get('arm_token')=='test-arm-ack', 6)
    set_phase('wrong')
    until('grant withdrawn if carried away', lambda: not read_payload(ready).get('ready') and not read_payload(ready).get('arm_token') and not measured.exists(), 4)
    assert proc.poll() is None
    trace('success', amcl_estimates=estimates(), seed_count=seeds())


if __name__ == '__main__':
    results = Path(tempfile.mkdtemp(prefix='tb3_start_validation_'))
    loopback(results)
    print(results, flush=True)
    run(results)
