"""Exercise the automatic start handshake without creating ROS nodes or moving hardware."""
import contextlib
import io
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest
from types import SimpleNamespace
from unittest.mock import Mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from auto_start_localization import StartMonitor, covariance_ok
from start_scan_match import MatchResult


class StartHandshakeTest(unittest.TestCase):
    def setUp(self):
        folder = self.enterContext(tempfile.TemporaryDirectory())
        self.enterContext(contextlib.redirect_stdout(io.StringIO()))
        folder = Path(folder)
        m = self.m = StartMonitor.__new__(StartMonitor)
        m.args = SimpleNamespace(output=folder/"pose.json", watch_status=folder/"status.json",
                                 countdown_request=folder/"countdown.json", arm_request=folder/"arm.json")
        m.generation = 1
        m.phase = m.reason = ""
        m.home = (.0, .0, -.7)
        m.stable_since = m.countdown_since = m.seed = m.seed_odom = None
        m.measured = m.candidate = m.last_good = m.amcl = None
        m.arm_token = m.pending_token = m.blocked_token = None
        m.matched_scan_stamp = 0
        self.fit = MatchResult(True, .02, -.01, math.pi, .93, .16, .94, "matched")
        # Fake only AMCL cross checks, which are verified in the isolated ROS integration.
        m.aligned = lambda _: {"generation": m.generation, "map_pose": [.02, -.01, math.pi]}
        m.send_seed = Mock(side_effect=lambda _: setattr(m, "seed", (2., 2., [.02, -.01, math.pi])))

    def scan(self, t, match=None):
        self.m.accept(match or self.fit, t, t)

    def tracking(self):
        for i in range(12):
            self.scan(i * .2)
        self.assertEqual(1, self.m.send_seed.call_count)
        self.assertEqual("tracking", self.m.phase)
        self.assertEqual(1, json.loads(self.m.args.output.read_text())["generation"])

    def ready(self):
        self.tracking()
        self.m.args.countdown_request.write_text('{"generation":1}')
        for i in range(12,28):
            self.scan(i*.2)
        self.assertEqual("ready", self.m.phase)

    def test_preparation_has_no_countdown_until_same_generation_requests(self):
        self.tracking()
        self.m.args.countdown_request.write_text('{"generation":3}')
        for i in range(12,35):
            self.scan(i*.2)
        self.assertEqual("tracking", self.m.phase)
        self.m.args.countdown_request.write_text('{"generation":1}')
        self.scan(7.)
        self.assertEqual("countdown", self.m.phase)
        for _ in range(20):
            self.scan(7.)  # Replaying a single scan does not advance the countdown.
        self.assertEqual("countdown", self.m.phase)

    def test_challenge_needs_generation_and_fresh_scan_then_resets_on_ambiguity(self):
        self.ready()
        m = self.m
        m.args.arm_request.write_text('{"token":"missing-gen"}')
        self.scan(5.6)
        self.assertIsNone(m.arm_token)
        m.args.arm_request.write_text('{"generation":1,"token":"A"}')
        self.scan(5.8)
        self.assertIsNone(m.arm_token)
        self.scan(6.0)
        self.assertEqual("A", m.arm_token)
        self.scan(6.2, MatchResult(False, None, None, None, .2, .01, .8, "ambiguous competing map pose"))
        m.publish_status(force=True)
        state = json.loads(m.args.watch_status.read_text())
        self.assertFalse(state["ready"])
        self.assertEqual(2,state["generation"])
        self.assertIsNone(state["arm_token"])
        self.assertFalse(m.args.output.exists())
        for i in range(32,65):
            self.scan(i*.2)
        self.assertEqual("tracking",m.phase)
        self.assertEqual(2,json.loads(m.args.output.read_text())["generation"])

    def test_motion_rejects_prior_countdown(self):
        self.ready()
        from nav_msgs.msg import Odometry
        msg=Odometry()
        msg.header.frame_id="odom"
        msg.child_frame_id="base_footprint"
        msg.pose.pose.orientation.w=1.
        msg.twist.twist.angular.z=.20
        self.m.odom=[]
        self.m.on_odom(msg)
        self.assertEqual("waiting",self.m.phase)
        self.assertEqual(2,self.m.generation)
        self.assertFalse(self.m.args.output.exists())

    def test_covariance_includes_joint_xy_uncertainty(self):
        good=[0.]*36
        good[0], good[7], good[35]=.02,.03,.03
        self.assertTrue(covariance_ok(good))
        correlated=good.copy()
        correlated[0],correlated[7],correlated[1],correlated[6]=.08,.08,.04,.04
        self.assertFalse(covariance_ok(correlated))
        good[35]=math.radians(30)**2
        self.assertFalse(covariance_ok(good))


if __name__ == "__main__":
    unittest.main()
