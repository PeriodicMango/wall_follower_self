#!/usr/bin/env python3
"""One-shot check of robot .103's mapping graph and actual sensor data, before arming."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSCompatibility, QoSProfile, qos_check_compatible, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CompressedImage, LaserScan
from tf2_ros import Buffer, TransformException, TransformListener


HERE = Path(__file__).resolve().parents[1]
OVERLAY = HERE.parent/'TurtleBot-Sim-2026-11-11/teammate_overlay/install'
DEFAULT_FOLLOWER = OVERLAY/'lib/wall_follower/wall_follower'
WALL = '/turtlebot_test/wall_cmd_vel'


def stamp(t):
    return t.sec + t.nanosec*1e-9


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))


def endpoint_name(info):
    return info.node_namespace.rstrip('/')+'/'+info.node_name


class Check(Node):
    def __init__(self, args):
        super().__init__('check_robot103_mapping_inputs', enable_rosout=False, start_parameter_services=False)
        self.args = args
        self.samples = {}
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.last_jpeg = 0.0
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(LaserScan, '/scan', self.scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.odom, qos_profile_sensor_data)
        self.create_subscription(CompressedImage, args.camera, self.image, qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid, '/map', self.grid, latched)
        self.create_subscription(PoseStamped, '/wall_follower/start_pose', self.start, latched)
        for topic in ('/cmd_vel', WALL):
            self.create_subscription(Twist, topic, lambda t, name=topic: self.velocity(name, t),
                                     qos_profile_sensor_data)

    def seen(self, kind, ok, payload, t=None):
        self.samples[kind] = dict(valid=bool(ok), detail=payload, at=time.monotonic(),
                                  stamp=stamp(t) if t is not None else None)

    def scan(self, m):
        valid = [v for v in m.ranges if math.isfinite(v) and m.range_min<=v<=m.range_max]
        self.seen('scan', bool(valid) and bool(m.header.frame_id),
                  dict(frame=m.header.frame_id, valid=len(valid), nearest_m=min(valid) if valid else None),
                  m.header.stamp)

    def odom(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        self.seen('odom', m.header.frame_id=='odom' and m.child_frame_id=='base_footprint'
                  and all(map(math.isfinite,[p.x,p.y,q.x,q.y,q.z,q.w])),
                  dict(frame=m.header.frame_id,child=m.child_frame_id,x=p.x,y=p.y,yaw=yaw(q)),m.header.stamp)

    def image(self, m):
        if time.monotonic()-self.last_jpeg < .25:
            return
        self.last_jpeg = time.monotonic()
        arr = np.frombuffer(m.data, dtype=np.uint8)
        try:
            image = cv2.imdecode(arr, cv2.IMREAD_COLOR) if len(arr)>2 and arr[0]==255 and arr[1]==216 else None
        except cv2.error:
            image = None
        self.seen('jpeg', image is not None,
                  dict(bytes=len(arr), width=int(image.shape[1]) if image is not None else 0,
                       height=int(image.shape[0]) if image is not None else 0, frame=m.header.frame_id), m.header.stamp)

    def grid(self, m):
        known = any(v>=0 for v in m.data)
        self.seen('map', m.header.frame_id=='map' and known and m.info.resolution>0,
                  dict(width=m.info.width,height=m.info.height,resolution=m.info.resolution), m.header.stamp)

    def start(self, m):
        p,q=m.pose.position,m.pose.orientation
        finite=all(map(math.isfinite,[p.x,p.y,p.z,q.x,q.y,q.z,q.w]))
        self.seen('start_pose', m.header.frame_id=='odom' and finite and stamp(m.header.stamp)>0
                  and abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1)<.03,
                  dict(frame=m.header.frame_id,x=p.x,y=p.y,yaw=yaw(q)), m.header.stamp)

    def velocity(self, name, m):
        self.seen(name, math.isfinite(m.linear.x) and math.isfinite(m.angular.z), [m.linear.x, m.angular.z])

    def check(self):
        guard = '/turtlebot_test_velocity_guard'
        expected = {'/scan':'/hlds_laser_publisher', '/odom':'/diff_drive_controller', '/map':'/cartographer_occupancy_grid_node',
                    '/cmd_vel':guard, WALL:self.args.follower_node, '/wall_follower/start_pose':self.args.follower_node,
                    '/wall_follower/finished':self.args.follower_node, self.args.camera:'/hardware_camera_jpeg',
                    '/marker_position':'/see_marker'}
        source_infos = {topic:self.get_publishers_info_by_topic(topic) for topic in expected}
        pubs = {topic:list(map(endpoint_name,infos)) for topic,infos in source_infos.items()}
        bad = [f'{t}: expected single publisher {name}, saw {pubs[t]}' for t,name in expected.items()
               if pubs[t]!=[name]]
        subs = {}
        required = {'/cmd_vel':['/turtlebot3_node'], WALL:[guard],self.args.camera:['/see_marker'],
                    '/scan':[self.args.follower_node], '/odom':[self.args.follower_node],
                    '/marker_position':['/point_transformer'],
                    '/wall_follower/start_pose':['/point_transformer'],'/wall_follower/finished':['/point_transformer']}
        if self.args.follower_binary.name == 'wall_follower_3':
            required['/marker_position'].append(self.args.follower_node)
        for topic, targets in required.items():
            readers=self.get_subscriptions_info_by_topic(topic)
            subs[topic]=list(map(endpoint_name,readers))
            for target in targets:
                matching=[r for r in readers if endpoint_name(r)==target]
                if len(matching)!=1:
                    bad.append(f'{topic}: expected one subscription by {target}')
                elif len(source_infos[topic])==1:
                    writer=source_infos[topic][0]
                    reader=matching[0]
                    compatible,reason=qos_check_compatible(writer.qos_profile,reader.qos_profile)
                    if reader.topic_type!=writer.topic_type or compatible==QoSCompatibility.ERROR:
                        bad.append(f'{topic}: incompatible subscription by {target} ({reason or "message type"})')
        details = {}
        now=time.monotonic()
        for k, max_age in [('scan',.6),('odom',.6),('map',6),('jpeg',2),('start_pose',None),('/cmd_vel',.6),(WALL,.3)]:
            s = self.samples.get(k)
            if s:
                details[k]=dict(detail=s['detail'], valid=s['valid'], age_s=now-s['at'], stamp=s['stamp'])
            if not s or not s['valid'] or (max_age is not None and now-s['at']>=max_age):
                bad.append(f'{k}: valid recent sample missing')
            elif k in ('scan','odom','jpeg') and abs(time.time()-s['stamp']) > 2:
                bad.append(f'{k}: sensor stamp differs from host wall clock by >2 s')
        delta = None
        if all(self.samples.get(k,{}).get('valid') for k in ('odom','start_pose')):
            start=self.samples['start_pose']; odom=self.samples['odom']
            s=start['detail']; o=odom['detail']
            delta={'xy_m':math.hypot(s['x']-o['x'],s['y']-o['y']),
                   'yaw_rad':abs(math.remainder(s['yaw']-o['yaw'],2*math.pi))}
            if start['stamp']>odom['stamp']+.5 or delta['xy_m']>.04 or delta['yaw_rad']>.15:
                bad.append('Latched start pose does not match current physical odometry while disarmed')
        tf_pose = None
        try:
            tf = self.tf.lookup_transform('map','base_footprint',Time())
            age = time.time()-stamp(tf.header.stamp)
            p=tf.transform.translation
            tf_pose=dict(x=p.x,y=p.y,stamp=stamp(tf.header.stamp),age_s=age)
            if abs(age)>2 or not all(map(math.isfinite,[p.x,p.y])):
                bad.append('map→base_footprint transform stale or invalid')
        except TransformException:
            bad.append('map→base_footprint transform unavailable')
        return dict(errors=bad,publishers=pubs,subscribers=subs,samples=details,
                    start_vs_odom=delta,map_base=tf_pose)


def local_follower(binary, expected_digest=None):
    binary = binary.resolve(strict=True)
    standalone = (HERE/'local_variants/wall_follower_2_3/build/wall_follower_2_3').resolve()
    actual = standalone.parent if binary == standalone else Path(get_package_prefix('wall_follower')).resolve()
    digest = hashlib.sha256(binary.read_bytes()).hexdigest()
    lines = subprocess.check_output(['ps','-axo','command='], text=True, timeout=2).splitlines()
    matches = sum(line.strip()==str(binary) or line.strip().startswith(str(binary)+' ') for line in lines)
    return dict(expected_binary=str(binary), sha256=digest, active_prefix=str(actual), matching_processes=matches,
                correct=((binary == standalone or actual == OVERLAY.resolve()) and os.access(binary,os.X_OK) and matches==1
                         and (expected_digest is None or expected_digest.lower()==digest)))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--timeout',type=float,default=15.0)
    p.add_argument('--output',type=Path)
    p.add_argument('--follower-node',default='/wall_follower_node')
    p.add_argument('--follower-binary', type=Path, default=DEFAULT_FOLLOWER,
                   help='Absolute path of the exact compiled executable launched in this run')
    p.add_argument('--follower-sha256',help='Optional executable SHA-256 from a pinned build manifest')
    p.add_argument('--camera',default='/turtlebot_test/camera/compressed')
    a=p.parse_args()
    if not math.isfinite(a.timeout) or a.timeout<=0:
        p.error('timeout must be positive')
    if os.getenv('ROS_DOMAIN_ID')!='3':
        p.exit(1,'Expected robot .103 environment with ROS_DOMAIN_ID=3\n')
    rclpy.init()
    node=Check(a)
    try:
        deadline=time.monotonic()+a.timeout
        result={}
        while rclpy.ok():
            until=min(deadline,time.monotonic()+.35)
            while time.monotonic()<until:
                rclpy.spin_once(node,timeout_sec=min(.05,max(0,until-time.monotonic())))
            result=node.check()
            try:
                result['follower_process']=local_follower(a.follower_binary,a.follower_sha256)
                if not result['follower_process']['correct']:
                    result['errors'].append(f'Expected one matching process for {a.follower_binary} in the selected environment')
            except (OSError,LookupError,subprocess.SubprocessError) as exc:
                result['errors'].append('Could not verify follower binary: '+str(exc))
            if not result['errors'] or time.monotonic()>=deadline:
                break
        result['ok']=not result.get('errors', ['No observation'])
        result['updated_unix']=time.time()
        text=json.dumps(result,indent=2)
        if a.output:
            a.output.parent.mkdir(parents=True,exist_ok=True)
            t=a.output.with_suffix('.tmp')
            t.write_text(text+'\n'); t.replace(a.output)
        print('Mapping inputs ready.' if result['ok'] else '\n'.join(result.get('errors',['No observation'])))
        return 0 if result['ok'] else 1
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__=='__main__':
    sys.exit(main())
