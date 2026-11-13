import csv
from itertools import permutations
from pathlib import Path
import sys
import tempfile
import unittest

BASE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(BASE / "scripts"))
from exam_marker_order import create_order_template, read_exam_order


class ExamOrderTests(unittest.TestCase):
    def setUp(self):
        folder = BASE / "runs/Real_103_2026-11-11_152732/maps"
        with (folder / "markers_Real_103_2026-11-11_152732.txt").open(newline="") as stream:
            rows = list(csv.reader(stream))
        self.markers = [(float(x), float(y), kind) for x, y, kind in rows[1:]]
        with (folder / "landmarks.csv").open(newline="") as stream:
            self.original = list(csv.reader(stream))
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.csv = Path(self.temp.name) / "顺序.csv"

    def test_all_real_numeric_row_permutations_preserve_coordinates(self):
        known = {kind: (x, y, kind) for x, y, kind in self.markers}
        id_to_kind = {"0": "yellow/pink", "2": "blue/pink", "4": "pink/green"}
        for order in permutations(self.original):
            with self.csv.open("w", newline="") as stream:
                csv.writer(stream).writerows(order)
            self.assertEqual(read_exam_order(self.csv, self.markers),
                             [known[id_to_kind[row[2]]] for row in order])

    def test_chinese_order_and_template_preservation(self):
        create_order_template(self.csv, self.markers)
        self.assertEqual(read_exam_order(self.csv, self.markers), self.markers)
        self.csv.write_text("marker\r\n粉色／绿色\r\n蓝/粉\r\nyellow/pink\r\n", encoding="utf-8-sig")
        self.assertEqual(read_exam_order(self.csv, self.markers), self.markers[::-1])
        with self.assertRaises(FileExistsError):
            create_order_template(self.csv, self.markers)
        self.assertEqual(read_exam_order(self.csv, self.markers), self.markers[::-1])

    def test_four_measured_markers_require_all_four(self):
        markers = self.markers + [(0.5, -0.3, "green/pink")]
        self.csv.write_text("marker\ngreen/pink\npink/green\nblue/pink\nyellow/pink\n")
        self.assertEqual(read_exam_order(self.csv, markers), markers[::-1])
        self.csv.write_text("marker\npink/green\nblue/pink\nyellow/pink\n")
        with self.assertRaisesRegex(ValueError, "green/pink"):
            read_exam_order(self.csv, markers)

    def test_refuse_bad_identity_or_coordinate_instead_of_driving(self):
        for contents in (
            "marker\nyellow/pink\nyellow/pink\npink/green\n",
            "marker\nyellow/pink\nblue/pink\n",
            "marker\nyellow/pink\nblue/pink\npink/green\ngreen/pink\n",
            "1.704,-0.101,2\n",
            "nan,0.0,0\n",
            "marker\nyellow/pink,blue/pink\npink/green\n",
            "marker\n",
        ):
            with self.subTest(contents=contents):
                self.csv.write_text(contents)
                with self.assertRaises(ValueError):
                    read_exam_order(self.csv, self.markers)


if __name__ == "__main__":
    unittest.main()
