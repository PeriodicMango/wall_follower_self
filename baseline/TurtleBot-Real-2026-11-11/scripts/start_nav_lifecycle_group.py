"""Start one named Nav2 lifecycle group and check its typed response."""
import argparse
import time

import rclpy
from nav2_msgs.srv import ManageLifecycleNodes


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("group", choices=("localization", "navigation"))
    args = parser.parse_args()
    manager = "lifecycle_manager_" + args.group
    service = "/" + manager + "/manage_nodes"
    rclpy.init()
    node = rclpy.create_node("robot103_start_" + args.group, enable_rosout=False, start_parameter_services=False)
    try:
        client = node.create_client(ManageLifecycleNodes, service)
        deadline = time.monotonic() + 40
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
            n = node.get_node_names_and_namespaces().count((manager, "/"))
            if n > 1:
                raise RuntimeError(f"{service} 对应多个管理节点")
            if n == 1 and client.service_is_ready():
                break
        else:
            raise TimeoutError(f"{service} 未就绪")
        request = ManageLifecycleNodes.Request(command=ManageLifecycleNodes.Request.STARTUP)
        print(f"{service} STARTUP", flush=True)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=60)
        if not future.done() or not future.result() or not future.result().success:
            raise RuntimeError(f"{service} 未确认成功激活节点")
        print(f"{service} success=true", flush=True)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
