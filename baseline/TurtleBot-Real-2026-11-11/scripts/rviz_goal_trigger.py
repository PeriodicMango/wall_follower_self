"""Use one new RViz SetGoal click as the start button for the saved CSV route."""
import math
import time

TRIGGER_TOPIC = "/robot103/manual_start_goal"
RVIZ_NODE = "/robot103_manual_nav_rviz"


def wait_for_click(nav):
    """Wait on the existing navigator. The arrow is a button, never a Nav2 goal."""
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

    log = nav.get_logger()
    ready_ns = None
    expected_gid = None
    accepted = False

    def source():
        pubs = nav.get_publishers_info_by_topic(TRIGGER_TOPIC)
        if len(pubs) != 1:
            return None
        p = pubs[0]
        if (p.node_namespace.rstrip("/") + "/" + p.node_name != RVIZ_NODE
                or p.topic_type != "geometry_msgs/msg/PoseStamped"):
            return None
        return bytes(p.endpoint_gid)

    def click(msg):
        nonlocal accepted
        if ready_ns is None or source() != expected_gid:
            return
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        age = nav.get_clock().now().nanoseconds - stamp
        p, q = msg.pose.position, msg.pose.orientation
        if (msg.header.frame_id != "map" or stamp < ready_ns or not 0 <= age <= 2_000_000_000
                or not all(map(math.isfinite, (p.x, p.y, p.z, q.x, q.y, q.z, q.w)))
                or abs(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w - 1.) > .03):
            log.warn("发车箭头已过期或不在 map 坐标系，请重新点选。")
            return
        accepted = True

    sub = nav.create_subscription(PoseStamped, TRIGGER_TOPIC, click,
                                  QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE,
                                             reliability=ReliabilityPolicy.RELIABLE))
    try:
        log.info("定位已确认，正在连接本轮 RViz 的 2D Goal Pose 发车按钮。")
        stable_since = time.monotonic()
        while rclpy.ok(context=nav.context):
            gid = source()
            if gid != expected_gid:
                expected_gid, ready_ns, stable_since = gid, None, time.monotonic()
            if gid and ready_ns is None and time.monotonic() - stable_since >= .5:
                ready_ns = nav.get_clock().now().nanoseconds
                log.info("[RVIZ_TRIGGER_READY] 可以点击：选 RViz 的 2D Goal Pose，在地图上拖出一次箭头，即按 CSV 顺序访问地标并返回起点。")
            rclpy.spin_once(nav, timeout_sec=.05)
            if accepted:
                log.info("[RVIZ_TRIGGERED] 已收到发车点击，正在按 CSV 顺序导航。")
                return
        raise RuntimeError("ROS 已关闭，未收到发车点击。")
    finally:
        nav.destroy_subscription(sub)
