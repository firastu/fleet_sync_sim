"""Pure-Python tests for the stage 1 export loader/validator (ADR-020).

Runs without Isaac Sim: the loader is the CPU-side contract the Isaac viewer
consumes.
"""

import json
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from replay import ReplayDataError, load_export, validate_export


def write_export(directory: Path, manifest: dict, snapshots: list) -> Path:
    directory.mkdir(parents=True, exist_ok=True)
    (directory / "manifest.json").write_text(json.dumps(manifest), encoding="utf-8")
    with (directory / "poses.jsonl").open("w", encoding="utf-8") as poses:
        for snapshot in snapshots:
            poses.write(json.dumps(snapshot) + "\n")
    return directory


def make_manifest(**overrides) -> dict:
    manifest = {
        "schema": "fleet-isaac-replay/1",
        "kind": "readonly-replay-export",
        "scenario_name": "synthetic",
        "resolved_seed": "7",
        "scenario_hash": "fnv1a64:0000000000000001",
        "map_hash": "fnv1a64:0000000000000002",
        "origin": {"latitude_deg": 52.370, "longitude_deg": 9.730},
        "scene_frame": {"axes": "enu", "translation_units": "meters", "z_up": True},
        "heading_convention": "radians_clockwise_from_north",
        "footprint_radius_m": 50.0,
        "observed_extent_m": 30.1,
        "tick_unit_ms": 1,
        "first_tick_ms": "0",
        "final_tick_ms": "2000",
        "snapshot_period_ms": "1000",
        "snapshot_count": 3,
        "robots": [{"robot_id": 1, "name": "robot_a"},
                   {"robot_id": 2, "name": "robot_b"}],
    }
    manifest.update(overrides)
    return manifest


def make_snapshots(with_belief_none=False) -> list:
    snapshots = []
    for index, tick in enumerate((0, 1000, 2000)):
        belief = None if (with_belief_none and index == 0) else {
            "latitude_deg": 52.3701 + 0.00001 * index,
            "longitude_deg": 9.7301,
            "heading_rad": 0.0,
            "estimated_at_ms": str(tick),
            "dead_reckoned": False,
        }
        snapshots.append({
            "schema": "fleet-isaac-replay/1",
            "tick_ms": str(tick),
            "robots": [
                {"robot_id": 1, "name": "robot_a",
                 "truth": {"latitude_deg": 52.370 + 0.00001 * index,
                           "longitude_deg": 9.730 + 0.00001 * index,
                           "heading_rad": 0.1 * index},
                 "belief": belief},
                {"robot_id": 2, "name": "robot_b",
                 "truth": {"latitude_deg": 52.3699,
                           "longitude_deg": 9.7299,
                           "heading_rad": 1.0},
                 "belief": None},
            ],
        })
    return snapshots


class LoadExportTest(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        self.root = Path(self._tmp.name)

    def test_loads_and_parses_ticks_and_order(self):
        directory = write_export(self.root / "ok", make_manifest(), make_snapshots())
        data = load_export(directory)
        self.assertEqual([int(s["tick_ms"]) for s in data["snapshots"]], [0, 1000, 2000])
        self.assertEqual(data["manifest"]["final_tick_ms"], 2000)

        summary = validate_export(directory)
        self.assertEqual(summary["snapshots"], 3)
        self.assertEqual(summary["robots"], 2)
        # 2 robots x (truth + non-null beliefs): robot 1 belief on 3 ticks,
        # robot 2 belief never present -> 3*2 + 3 = 9.
        self.assertEqual(summary["poses_converted"], 9)

    def test_null_belief_is_carried_not_filled(self):
        directory = write_export(self.root / "nullbelief", make_manifest(),
                                 make_snapshots(with_belief_none=True))
        data = load_export(directory)
        self.assertIsNone(data["snapshots"][0]["robots"][0]["belief"])
        self.assertIsNotNone(data["snapshots"][1]["robots"][0]["belief"])

    def test_wrong_schema_rejected(self):
        directory = write_export(self.root / "schema", make_manifest(schema="other/9"),
                                 make_snapshots())
        with self.assertRaises(ReplayDataError):
            load_export(directory)

    def test_non_ascending_ticks_rejected(self):
        snapshots = make_snapshots()
        snapshots[2]["tick_ms"] = "500"  # goes backward
        directory = write_export(self.root / "ticks", make_manifest(), snapshots)
        with self.assertRaises(ReplayDataError):
            load_export(directory)

    def test_robot_set_mismatch_rejected(self):
        snapshots = make_snapshots()
        snapshots[1]["robots"] = snapshots[1]["robots"][:1]  # robot 2 missing
        directory = write_export(self.root / "missing", make_manifest(), snapshots)
        with self.assertRaises(ReplayDataError):
            load_export(directory)

    def test_unsorted_manifest_robots_rejected(self):
        manifest = make_manifest(robots=[{"robot_id": 2, "name": "b"},
                                         {"robot_id": 1, "name": "a"}])
        directory = write_export(self.root / "unsorted", manifest, [])
        with self.assertRaises(ReplayDataError):
            load_export(directory)

    def test_out_of_footprint_pose_rejected_by_validate(self):
        snapshots = make_snapshots()
        snapshots[1]["robots"][0]["truth"]["latitude_deg"] = 52.372  # ~222 m out
        directory = write_export(self.root / "far", make_manifest(), snapshots)
        with self.assertRaises(Exception):  # noqa: B017 (ConversionError subclass)
            validate_export(directory)

    def test_missing_manifest_rejected(self):
        with self.assertRaises(ReplayDataError):
            load_export(self.root / "does-not-exist")


if __name__ == "__main__":
    unittest.main()
