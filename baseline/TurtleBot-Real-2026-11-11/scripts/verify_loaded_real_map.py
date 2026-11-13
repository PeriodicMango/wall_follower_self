#!/usr/bin/env python3
"""Check the live map server's YAML and occupancy grid without initializing localization."""
import argparse
import json
import math
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
import yaml
from nav_msgs.msg import OccupancyGrid
from rcl_interfaces.srv import GetParameters
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from check_navigation_ready import expected_grid


def compare(message, meta, expected):
    info = message.info
    p, q = info.origin.position, info.origin.orientation
    theta = float(meta["origin"][2])
    # q and -q encode the same rotation. Map server loads planar YAML origins.
    quat = np.array([q.x, q.y, q.z, q.w])
    target = np.array([0.0, 0.0, math.sin(theta / 2), math.cos(theta / 2)])
    yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
    actual = np.asarray(message.data, dtype=np.int8)
    dims_match = (info.height, info.width) == expected.shape
    matched = int(np.count_nonzero(actual == expected.ravel())) if dims_match and actual.size == expected.size else None
    geometry = (message.header.frame_id == "map" and dims_match and abs(info.resolution - meta["resolution"]) < 1e-6
                and np.allclose([p.x, p.y, p.z], [*meta["origin"][:2], 0.], rtol=0., atol=1e-5)
                and min(np.max(abs(quat - target)), np.max(abs(quat + target))) < 1e-5)
    return {"width": info.width, "height": info.height, "resolution": info.resolution,
            "origin_xy_yaw": [p.x, p.y, yaw], "frame": message.header.frame_id,
            "cells": int(actual.size), "matched_cells": matched, "geometry_matches": bool(geometry),
            "all_cells_match": matched == expected.size}


class MapCheck(Node):
    def __init__(self, path):
        super().__init__("verify_loaded_real_map", enable_rosout=False, start_parameter_services=False)
        self.path = path
        self.meta, self.expected = expected_grid(path)
        self.actual = self.filename = self.pending = None
        self.next_request = 0.0
        self.param = self.create_client(GetParameters, "/map_server/get_parameters")
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, "/map", self.on_map, qos)

    def on_map(self, msg):
        self.actual = compare(msg, self.meta, self.expected)

    def report(self):
        now = time.monotonic()
        if self.pending is not None and self.pending.done():
            response = self.pending.result()
            self.filename = response.values[0].string_value if response and response.values else None
            self.pending = None
        if self.pending is None and now >= self.next_request and self.param.service_is_ready():
            self.pending = self.param.call_async(GetParameters.Request(names=["yaml_filename"]))
            self.next_request = now + 1.0
        pubs = [p.node_namespace.rstrip("/") + "/" + p.node_name for p in self.get_publishers_info_by_topic("/map")]
        servers = self.get_node_names_and_namespaces().count(("map_server", "/"))
        file_matches = bool(self.filename and Path(self.filename).is_absolute() and Path(self.filename).resolve() == self.path)
        errors = []
        if pubs != ["/map_server"] or servers != 1:
            errors.append("Expected one /map_server node and its sole /map publisher")
        if not file_matches:
            errors.append("Map server yaml_filename did not match the selected file")
        if self.actual is None:
            errors.append("No retained /map received")
        elif not (self.actual["geometry_matches"] and self.actual["all_cells_match"]):
            errors.append("Published geometry or occupancy cells differ")
        height, width = self.expected.shape
        return {"ok": not errors, "errors": errors, "submitted_yaml": str(self.path),
                "submitted": {"width": width, "height": height, "resolution": self.meta["resolution"],
                              "origin_xy_yaw": self.meta["origin"], "cells": int(self.expected.size)},
                "reported_yaml": self.filename, "filename_matches": file_matches, "actual": self.actual,
                "map_publishers": pubs, "map_server_node_count": servers}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map", type=Path, required=True)
    parser.add_argument("--timeout", type=float, default=20.0)
    args = parser.parse_args()
    if not math.isfinite(args.timeout) or args.timeout <= 0 or not args.map.is_absolute():
        parser.error("Provide an absolute map path and a finite, positive timeout")
    node = None
    rclpy.init()
    try:
        node = MapCheck(args.map.resolve(strict=True))
        deadline = time.monotonic() + args.timeout
        while True:
            rclpy.spin_once(node, timeout_sec=max(0., min(.15, deadline - time.monotonic())))
            result = node.report()
            if result["ok"] or time.monotonic() >= deadline:
                break
    except (KeyboardInterrupt, ExternalShutdownException, OSError, ValueError, TypeError, AttributeError,
            KeyError, RuntimeError, yaml.YAMLError) as exc:
        result = {"ok": False, "submitted_yaml": str(args.map), "errors": [str(exc) or "Interrupted"]}
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
    print(json.dumps(result, ensure_ascii=False), flush=True)
    return 0 if result["ok"] else 1


if __name__ == "__main__":
    sys.exit(main())
