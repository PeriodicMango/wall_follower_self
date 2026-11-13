#!/usr/bin/env python3
"""Capture a stationary AMCL pose following a new RViz 2D Pose Estimate on .103."""
import argparse
from collections import deque
import json
import math
import os
from pathlib import Path
import sys
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


def stamp(t):
    return t.sec + t.nanosec*1e-9


def pose(p,q):
    if not all(map(math.isfinite,(p.x,p.y,p.z,q.x,q.y,q.z,q.w))):
        return None
    if abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1)>.03:
        return None
    return [p.x,p.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))]


def close(a,b,distance,angle):
    return (math.hypot(a[0]-b[0],a[1]-b[1])<=distance and
            abs(math.remainder(a[2]-b[2],2*math.pi))<=angle)


class Capture(Node):
    def __init__(self):
        super().__init__('capture_robot103_rviz_pose',enable_rosout=False,start_parameter_services=False)
        self.tf=Buffer()
        self.listener=TransformListener(self.tf,self)
        self.start=None
        self.amcl=None
        self.odom=deque(maxlen=180)
        self.note='等待 RViz 里的 2D Pose Estimate。'
        # Volatile subscriptions take only new messages. AMCL's latched estimate
        # from before the operator clicked cannot satisfy this capture.
        self.create_subscription(PoseWithCovarianceStamped,'/initialpose',self.initial,10)
        self.create_subscription(PoseWithCovarianceStamped,'/amcl_pose',self.estimate,10)
        self.create_subscription(Odometry,'/odom',self.odometry,qos_profile_sensor_data)

    def initial(self,m):
        self.start=self.amcl=None
        names=[i.node_namespace.rstrip('/')+'/'+i.node_name for i in self.get_publishers_info_by_topic('/initialpose')]
        if len(names)!=1 or 'rviz' not in names[0].lower():
            self.note=f'/initialpose 需要由本次 RViz 单独发布，当前发布者 {names}。'
            return
        coords=pose(m.pose.pose.position,m.pose.pose.orientation)
        if m.header.frame_id!='map' or not coords or abs(time.time()-stamp(m.header.stamp))>3:
            self.note='点选位姿无效或过期；请在 RViz 地图上重新定位。'
            return
        self.start=(stamp(m.header.stamp),coords,names[0])
        self.note='已收到 RViz 位姿，等待 AMCL 与激光定位结果。'

    def estimate(self,m):
        if self.start is None or m.header.frame_id!='map':
            return
        t=stamp(m.header.stamp)
        if t<self.start[0]-.15 or abs(time.time()-t)>2:
            return
        p=pose(m.pose.pose.position,m.pose.pose.orientation)
        cov=m.pose.covariance
        if (p is None or not all(map(math.isfinite,cov)) or min(cov[0],cov[7],cov[35])<0
            or not close(p,self.start[1],.75,math.radians(50))):
            self.amcl=None
            self.note='AMCL 与点选位置或朝向差异较大，或协方差无效；请核对地图后重新点选。'
            return
        self.amcl=(t,p,[cov[0],cov[7],cov[35]])

    def odometry(self,m):
        p=pose(m.pose.pose.position,m.pose.pose.orientation)
        v=m.twist.twist
        if p and m.header.frame_id=='odom' and m.child_frame_id=='base_footprint':
            self.odom.append((time.monotonic(),m.header.stamp,p,math.hypot(v.linear.x,v.linear.y),abs(v.angular.z)))

    def result(self):
        if not (self.start and self.amcl):
            return None
        odom_infos=self.get_publishers_info_by_topic('/odom')
        amcl_infos=self.get_publishers_info_by_topic('/amcl_pose')
        if (len(odom_infos)!=1 or odom_infos[0].node_name!='diff_drive_controller' or
            len(amcl_infos)!=1 or amcl_infos[0].node_name!='amcl'):
            self.note='等待真机里程计和单一 AMCL 发布者。'
            return None
        now=time.monotonic()
        recent=[s for s in self.odom if now-s[0]<1.7]
        if (not recent or recent[-1][0]-recent[0][0]<1.1 or now-recent[-1][0]>.4 or
            any(not (s[3]<.025 and s[4]<.04 and close(s[2],recent[0][2],.02,.04)) for s in recent) or
            any(b[0]-a[0]>.6 for a,b in zip(recent,recent[1:]))):
            self.note='请让机器人静止，等待连续的真机里程计读数。'
            return None
        for _,ros_stamp,odom_pose,_,_ in reversed(recent):
            t=stamp(ros_stamp)
            if abs(time.time()-t)>.6 or t<self.amcl[0]-.15:
                continue
            try:
                tf=self.tf.lookup_transform('map','base_footprint',Time.from_msg(ros_stamp))
            except TransformException:
                continue
            p=pose(tf.transform.translation,tf.transform.rotation)
            if p and close(p,self.amcl[1],.17,.20):
                return dict(stamp=t, map_pose=p, odom_pose=odom_pose,
                            odom_publisher_gid=bytes(odom_infos[0].endpoint_gid).hex(),
                            source='rviz_initialpose_amcl', initialpose_stamp=self.start[0],
                            rviz_node=self.start[2], initial_map_pose=self.start[1],
                            amcl_stamp=self.amcl[0], amcl_map_pose=self.amcl[1],amcl_xyyaw_variances=self.amcl[2])
        self.note='等待与静止里程计同一时刻、且与 AMCL 一致的 map → base_footprint。'
        return None


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--timeout',type=float,default=600)
    args=parser.parse_args()
    if not math.isfinite(args.timeout) or args.timeout<=0 or os.getenv('ROS_DOMAIN_ID')!='3':
        parser.error('请使用真机 Domain 3 和正数超时时间')
    args.output.parent.mkdir(parents=True,exist_ok=True)
    rclpy.init()
    node=Capture()
    try:
        print('保持机器人静止。在 RViz 选 2D Pose Estimate，在地图上的实际车身位置按下鼠标，'
              '朝真实车头方向拖出箭头并松开。核对红色激光点与墙体是否重合。',flush=True)
        deadline=time.monotonic()+args.timeout
        previous=''
        while rclpy.ok() and time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.1)
            data=node.result()
            if data:
                tmp=args.output.with_name(args.output.name+'.tmp')
                tmp.write_text(json.dumps(data,indent=2,allow_nan=False)+'\n')
                tmp.replace(args.output)
                print('已保存人工指定、经 AMCL 确认的静止位姿：'+str(args.output),flush=True)
                return 0
            if node.note!=previous:
                print(node.note,flush=True)
                previous=node.note
        print('等待定位超时：'+node.note,file=sys.stderr,flush=True)
        return 1
    except (KeyboardInterrupt,ExternalShutdownException):
        return 1
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__=='__main__':
    sys.exit(main())
