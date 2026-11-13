"""Exercise the teacher mission without creating ROS nodes or sending commands."""
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from builtin_interfaces.msg import Time
from nav2_simple_commander.robot_navigator import TaskResult

REAL = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REAL / "scripts"))
import navigate_saved_markers as mission


class FakeNavigator:
    def __init__(self, outcomes):
        self.outcomes = outcomes
        self.goals = []
        self.logs = []
        self.localizer = None
        self.nav_to_pose_client = SimpleNamespace(wait_for_server=lambda **k: True)

    def waitUntilNav2Active(self, *, localizer):
        self.localizer = localizer

    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=10**9, to_msg=lambda: Time(sec=1)))

    def get_logger(self):
        return SimpleNamespace(info=self.logs.append)

    def getPath(self, start, goal):
        return SimpleNamespace(poses=[start, goal])

    def goToPose(self, pose):
        self.goals.append(pose)
        return self.outcomes[len(self.goals) - 1] is not None

    def isTaskComplete(self):
        return True

    def getResult(self):
        return self.outcomes[len(self.goals) - 1]

    def getFeedback(self):
        return None


class TeacherRouteTest(unittest.TestCase):
    route = [(1., 2., "blue/pink"), (-1., 2., "yellow/pink")]
    home = (.35, -.42, 1.73)
    start = (.37, -.45, -2.1)

    def run_route(self, nav):
        with patch.object(mission, "current_map_xy", return_value=self.start[:2]):
            mission.visit(nav, self.route, self.start, home=self.home)

    def test_home_is_last_and_exact(self):
        nav = FakeNavigator([TaskResult.SUCCEEDED] * 3)
        self.run_route(nav)
        self.assertEqual(nav.localizer, "map_server")
        self.assertEqual(len(nav.goals), 3)
        for goal, (x, y, _) in zip(nav.goals, self.route):
            self.assertAlmostEqual(math.hypot(goal.pose.position.x - x, goal.pose.position.y - y), .4)
        home = nav.goals[-1]
        self.assertEqual(home.header.frame_id, "map")
        self.assertEqual((home.pose.position.x, home.pose.position.y), self.home[:2])
        self.assertAlmostEqual(2 * math.atan2(home.pose.orientation.z, home.pose.orientation.w), self.home[2])
        self.assertIn("[HOME_GOAL_ACCEPTED]", nav.logs[-3])
        self.assertIn("[HOME_REACHED]", nav.logs[-2])
        self.assertIn("[EXAM_COMPLETE]", nav.logs[-1])

    def test_manual_start_visits_three_in_order_then_returns_once(self):
        nav = FakeNavigator([TaskResult.SUCCEEDED] * 4)
        markers = [(-2., 1., "pink/green"), (1., -1., "yellow/pink"), (1., 2., "blue/pink")]
        departure = (-1.1, -.6, -2.1)  # Operator supplied a valid pose far from the recorded home.
        click = Mock(side_effect=lambda node: self.assertEqual(node.goals, []))
        with patch.object(mission, "current_map_xy", return_value=departure[:2]) as live_pose:
            mission.visit(nav, markers, departure, home=self.home, manual_start=True, before_first_goal=click)
        live_pose.assert_called_with(nav)
        click.assert_called_once_with(nav)
        self.assertEqual(len(nav.goals), 4)
        for goal, (x, y, _) in zip(nav.goals[:-1], markers):
            self.assertAlmostEqual(math.hypot(goal.pose.position.x - x, goal.pose.position.y - y), .40)
        home = nav.goals[-1]
        self.assertEqual(home.header.frame_id, "map")
        self.assertEqual((home.pose.position.x, home.pose.position.y), self.home[:2])
        self.assertAlmostEqual(2 * math.atan2(home.pose.orientation.z, home.pose.orientation.w), self.home[2])
        self.assertEqual(sum("[EXAM_COMPLETE]" in line for line in nav.logs), 1)

    def test_manual_start_cancelled_before_departure_leaves_all_goals_unsent(self):
        nav = FakeNavigator([])
        click = Mock(side_effect=InterruptedError("operator cancelled"))
        with patch.object(mission, "current_map_xy", return_value=self.start[:2]):
            with self.assertRaises(InterruptedError):
                mission.visit(nav, self.route, self.start, home=self.home, manual_start=True, before_first_goal=click)
        click.assert_called_once_with(nav)
        self.assertEqual(nav.goals, [])

    def test_home_and_marker_failures(self):
        for outcomes in ([None], [TaskResult.CANCELED],
                         [TaskResult.SUCCEEDED, TaskResult.SUCCEEDED, None],
                         [TaskResult.SUCCEEDED, TaskResult.SUCCEEDED, TaskResult.FAILED]):
            with self.subTest(outcomes=outcomes):
                nav = FakeNavigator(outcomes)
                with self.assertRaises(RuntimeError):
                    self.run_route(nav)
                self.assertEqual(len(nav.goals), len(outcomes))
                self.assertFalse(any("[EXAM_COMPLETE]" in line or "[HOME_REACHED]" in line for line in nav.logs))

    def test_start_or_live_position_outside_tolerance(self):
        for captured, live in [((0., 0., 0.), self.home[:2]), (self.start, (5., 1.))]:
            nav = FakeNavigator([])
            with patch.object(mission, "current_map_xy", return_value=live), self.assertRaises(ValueError):
                mission.visit(nav, self.route, captured, home=self.home)
            self.assertFalse(nav.goals)

    def test_unavailable_home_action(self):
        nav = FakeNavigator([TaskResult.SUCCEEDED] * 2)
        nav.nav_to_pose_client = SimpleNamespace(wait_for_server=lambda **k: False)
        with self.assertRaisesRegex(RuntimeError, "action unavailable"):
            self.run_route(nav)
        self.assertEqual(len(nav.goals), 2)
        self.assertFalse(any("[EXAM_COMPLETE]" in line for line in nav.logs))

    def test_list_only_reads_teacher_order_and_original_home(self):
        with tempfile.TemporaryDirectory() as tmp:
            d = Path(tmp)
            record = d / "markers.txt"
            record.write_text("50,75,1\n-1,2,yellow/pink\n1,2,blue/pink\n")
            home = record.with_suffix(".start_map.txt")
            home.write_text("0.35,-0.42,1.73\n")
            csv = d / "teacher.csv"
            csv.write_text("marker\nblue/pink\nyellow/pink\n")
            args = [sys.executable, str(REAL / "scripts/navigate_saved_markers.py"), "--list-only",
                    "--order-csv", str(csv), "--marker-file", str(record), "--return-to-start",
                    "--home-file", str(home)]
            preview = json.loads(subprocess.check_output(args))
            self.assertEqual([m["kind"] for m in preview["order"]], [m[2] for m in self.route])
            self.assertEqual(preview["home_map_pose"], list(self.home))
            self.assertEqual(subprocess.run(args + ["--shuffle"], capture_output=True).returncode, 2)
            self.assertEqual(subprocess.run(args[:-1] + [str(record)], capture_output=True).returncode, 2)
            home.unlink()
            self.assertEqual(subprocess.run(args, capture_output=True).returncode, 2)


if __name__ == "__main__":
    unittest.main()
