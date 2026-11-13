#!/usr/bin/env python3
"""Read-only navigation readiness on .103; optional wheel-odometry continuity watch."""
import argparse
from collections import deque
import json
import math
import os
from pathlib import Path
import sys
import time

import numpy as np
from PIL import Image
import rclpy
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid, Odometry
from rcl_interfaces.srv import GetParameters
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
import yaml

RAW = '/turtlebot_test/nav_cmd_vel'
SMOOTH = '/turtlebot_test/wall_cmd_vel'
SERVERS = ('map_server','amcl','controller_server','planner_server','bt_navigator','velocity_smoother',
           'behavior_server','smoother_server','waypoint_follower')


def write_json(path, value):
    if path:
        path.parent.mkdir(parents=True, exist_ok=True)
        t = path.with_suffix('.tmp')
        t.write_text(json.dumps(value, indent=2, allow_nan=False)+'\n')
        t.replace(path)


def wrap(angle):
    return math.remainder(angle, 2*math.pi)


def pose(m):
    p, q = m.pose.pose.position, m.pose.pose.orientation
    return [p.x,p.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))]


def stamp(h):
    return h.stamp.sec+h.stamp.nanosec*1e-9


def delta(a,b):
    return math.hypot(a[0]-b[0],a[1]-b[1]), abs(wrap(a[2]-b[2]))


def expected_grid(path):
    meta = yaml.safe_load(path.read_text())
    if meta.get('mode','trinary')!='trinary':
        raise ValueError('Expected the trinary map produced by the mapping script')
    gray = np.flipud(np.asarray(Image.open(path.parent/meta['image']).convert('L'), dtype=float))/255
    occupancy = gray if meta.get('negate',0) else 1-gray
    data = np.where(occupancy>meta['occupied_thresh'],100,np.where(occupancy<meta['free_thresh'],0,-1))
    return meta, data.astype(np.int8)


class Probe(Node):
    def __init__(self,a):
        super().__init__('robot103_odom_continuity' if a.watch else 'check_robot103_navigation',
                         enable_rosout=False,start_parameter_services=False)
        self.a = a
        self.history = deque(maxlen=1000)
        self.odom = self.scan = self.grid = None
        self.anchor = None
        self.faults = set()
        self.last_stamp = None
        self.max_delta = [0.,0.]
        self.watch_gid = None
        self.create_subscription(Odometry,'/odom',self.on_odom,qos_profile_sensor_data)
        if a.watch:
            return
        self.meta,self.expected = expected_grid(a.map)
        self.anchor = json.loads(a.handoff.read_text())
        for key in ('odom_pose','map_pose'):
            if len(self.anchor[key])!=3 or not all(map(math.isfinite,self.anchor[key])):
                raise ValueError('Invalid handoff '+key)
        self.create_subscription(OccupancyGrid,'/map',self.on_map,
                                 QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(LaserScan,'/scan',self.on_scan,qos_profile_sensor_data)
        self.probe_clients = {name:self.create_client(GetState,'/'+name+'/get_state') for name in SERVERS}
        self.probe_clients['yaml_filename'] = self.create_client(GetParameters,'/map_server/get_parameters')
        self.pending, self.state = {}, {}

    def on_map(self,m):
        origin = m.info.origin
        yaw = math.atan2(2*(origin.orientation.w*origin.orientation.z+origin.orientation.x*origin.orientation.y),
                         1-2*(origin.orientation.y**2+origin.orientation.z**2))
        self.grid = (m.header.frame_id=='map' and [m.info.height,m.info.width]==list(self.expected.shape)
                     and abs(m.info.resolution-self.meta['resolution'])<1e-6
                     and max(abs(a-b) for a,b in zip([origin.position.x,origin.position.y,yaw],self.meta['origin']))<1e-5
                     and np.array_equal(np.asarray(m.data,dtype=np.int8),self.expected.ravel()))

    def on_scan(self,m):
        valid=[v for v in m.ranges if math.isfinite(v) and m.range_min<=v<=m.range_max]
        self.scan=(time.monotonic(),stamp(m.header),min(valid) if valid else None)

    def on_odom(self,m):
        self.odom=(time.monotonic(),stamp(m.header),pose(m),m.header.frame_id,m.child_frame_id)
        self.history.append(self.odom)
        if self.a.watch and self.anchor:
            self.compare(self.odom)

    def compare(self,obs):
        _,tm,p,parent,child=obs
        if tm<self.anchor['stamp']:
            return
        if self.last_stamp is not None and (tm<self.last_stamp or tm-self.last_stamp>.65):
            self.faults.add('Odometry timestamps jumped or a gap exceeded 0.65 s')
        self.last_stamp=tm
        if parent!='odom' or child!='base_footprint' or not all(map(math.isfinite,p)):
            self.faults.add('Invalid physical odometry')
            return
        d=delta(p,self.anchor['odom_pose'])
        self.max_delta=[max(x,y) for x,y in zip(self.max_delta,d)]
        if d[0]>.03 or d[1]>.07:
            self.faults.add('Wheel odometry moved from the saved handoff')

    def odom_gid(self):
        infos=self.get_publishers_info_by_topic('/odom')
        return bytes(infos[0].endpoint_gid).hex() if len(infos)==1 and infos[0].node_name=='diff_drive_controller' else None

    def watch_tick(self):
        if not self.anchor and self.a.handoff.is_file():
            self.anchor=json.loads(self.a.handoff.read_text())
            if not self.history or self.history[0][1]>self.anchor['stamp']+.05:
                self.faults.add('No continuous recording at the handoff timestamp')
            for obs in self.history:
                self.compare(obs)
        if not self.anchor:
            return
        gid=self.odom_gid()
        if self.watch_gid is None:
            self.watch_gid=self.anchor.get('odom_publisher_gid') or gid
        if not gid or gid!=self.watch_gid:
            self.faults.add('Physical odometry publisher changed')
        if self.last_stamp is None or time.time()-self.last_stamp>.65:
            self.faults.add('Live odometry stopped during handoff')
        write_json(self.a.continuity,dict(updated_unix=time.time(),anchor_stamp=self.anchor['stamp'],
                   odom_publisher_gid=gid,issues=sorted(self.faults),max_translation_m=self.max_delta[0],
                   max_heading_change_rad=self.max_delta[1]))

    def poll_service_results(self):
        for n,c in self.probe_clients.items():
            if self.state.get(n)=='active' or (n=='yaml_filename' and self.state.get(n)):
                continue
            f=self.pending.get(n)
            if f is not None and f.done():
                r=f.result()
                if r is not None:
                    self.state[n]=(r.values[0].string_value if r.values else '') if n=='yaml_filename' else r.current_state.label
                self.pending.pop(n)
            elif f is None and c.service_is_ready():
                req=GetParameters.Request(names=['yaml_filename']) if n=='yaml_filename' else GetState.Request()
                self.pending[n]=c.call_async(req)

    def report(self):
        self.poll_service_results()
        errors=[]
        expected={'/map':['/map_server'],'/scan':['/hlds_laser_publisher'],'/odom':['/diff_drive_controller'],
                  '/cmd_vel':['/turtlebot_test_velocity_guard'],SMOOTH:['/velocity_smoother'],
                  RAW:['/behavior_server','/controller_server']}
        graph={}
        for t,names in expected.items():
            pubs=sorted(p.node_namespace.rstrip('/')+'/'+p.node_name for p in self.get_publishers_info_by_topic(t))
            graph[t]=pubs
            # Humble behaviors may create several publishers within the same server.
            if (sorted(set(pubs)) if t==RAW else pubs)!=names:
                errors.append('Unexpected publishers for '+t)
        for t,n in [(RAW,'velocity_smoother'),(SMOOTH,'turtlebot_test_velocity_guard'),('/cmd_vel','turtlebot3_node')]:
            if sum(p.node_name==n and p.node_namespace=='/' for p in self.get_subscriptions_info_by_topic(t))!=1:
                errors.append(f'{t} needs one /{n} subscriber')
        if self.grid is not True:
            errors.append('Published /map geometry or cell values not matched to selected map')
        if Path(self.state.get('yaml_filename','')).resolve()!=self.a.map.resolve():
            errors.append('Map server yaml_filename not verified')
        # The global costmap waits for AMCL's first user pose. Its activation
        # blocks the rest of the navigation lifecycle group until the mission
        # initializes AMCL. Before that, check configured graph and service presence.
        required_active=SERVERS if self.a.require_navigation_active else ('map_server','amcl')
        inactive=[n for n in required_active if self.state.get(n)!='active']
        if inactive:
            errors.append('Lifecycle nodes not active: '+', '.join(inactive))
        absent=[n for n in SERVERS if not self.probe_clients[n].service_is_ready()]
        if absent:
            errors.append('Missing lifecycle services: '+', '.join(absent))
        nodes=self.get_node_names_and_namespaces()
        ambiguous=[n for n in SERVERS if sum(name==n and ns=='/' for name,ns in nodes)!=1]
        if ambiguous:
            errors.append('Expected one instance of each lifecycle node: '+', '.join(ambiguous))
        scan_fresh=self.scan and time.monotonic()-self.scan[0]<.6 and abs(time.time()-self.scan[1])<2 and self.scan[2] is not None
        if not scan_fresh:
            errors.append('Fresh range-valid lidar required')
        endpoint=None
        if (not self.odom or time.monotonic()-self.odom[0]>=.6 or abs(time.time()-self.odom[1])>2
            or self.odom[3:]!=('odom','base_footprint') or not all(map(math.isfinite,self.odom[2]))):
            errors.append('Fresh physical odometry required')
        else:
            endpoint=delta(self.odom[2],self.anchor['odom_pose'])
            if self.odom[1]<self.anchor['stamp'] or endpoint[0]>.03 or endpoint[1]>.07:
                errors.append('Current odometry differs from the recorded handoff')
        gid=self.odom_gid()
        if self.anchor.get('odom_publisher_gid') and self.anchor['odom_publisher_gid']!=gid:
            errors.append('Odometry publisher changed after handoff')
        if self.a.continuity:
            try:
                record=json.loads(self.a.continuity.read_text())
                if (record['issues'] or abs(time.time()-record['updated_unix'])>2 or record['odom_publisher_gid']!=gid
                    or record['anchor_stamp']!=self.anchor['stamp']):
                    errors.append('Odometry continuity is interrupted, stale or from another handoff')
            except (OSError,ValueError,KeyError):
                errors.append('Continuity evidence unavailable')
        return dict(ok=not errors,errors=errors,state=self.state,graph=graph,odom_gid=gid,
                    handoff=str(self.a.handoff),endpoint_delta_xy_m_yaw_rad=endpoint,
                    scan_min_m=self.scan[2] if scan_fresh else None,updated_unix=time.time(),
                    continuity='monitored' if self.a.continuity else 'endpoint comparison only')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--map',type=Path)
    p.add_argument('--handoff',type=Path,required=True,help='Measured pose JSON from the mapping run')
    p.add_argument('--continuity',type=Path)
    p.add_argument('--watch',action='store_true',help='Sample wheel odom continuously, beginning before handoff capture')
    p.add_argument('--timeout',type=float,default=35)
    p.add_argument('--require-navigation-active',action='store_true',
                   help='After the mission has initialized AMCL, verify all lifecycle nodes are active')
    p.add_argument('--output',type=Path)
    a=p.parse_args()
    if (a.watch and not a.continuity) or (not a.watch and not a.map) or not math.isfinite(a.timeout) or a.timeout<=0:
        p.error('--watch requires --continuity; readiness requires --map and a positive timeout')
    if os.getenv('ROS_DOMAIN_ID')!='3':
        p.exit(1,'Expected ROS_DOMAIN_ID=3\n')
    rclpy.init()
    n=Probe(a)
    try:
        deadline=time.monotonic()+a.timeout
        while rclpy.ok():
            next_tick=time.monotonic()+.25
            while time.monotonic()<next_tick and rclpy.ok():
                rclpy.spin_once(n,timeout_sec=.04)
            if a.watch:
                n.watch_tick()
            else:
                result=n.report()
                if result['ok'] or time.monotonic()>=deadline:
                    write_json(a.output,result)
                    print('Guarded navigation ready.' if result['ok'] else '\n'.join(result['errors']))
                    return 0 if result['ok'] else 1
    except (KeyboardInterrupt,ExternalShutdownException):
        return 1
    finally:
        n.destroy_node()
        rclpy.try_shutdown()


if __name__=='__main__':
    sys.exit(main())
