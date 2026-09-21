#!/usr/bin/env python3
"""Isaac Sim stage 1 read-only replay viewer (ADR-020).

Consumes an export directory produced by the native `fleet_isaac_export`
adapter (manifest.json + poses.jsonl) and plays it back inside Isaac Sim as
non-physical display markers: one cube per robot for simulation truth, one
sphere for robot-local belief. Markers are kinematic — dynamics are disabled
for replay objects — and playback advances by RECORDED TICKS, never by
rendering FPS.

Two modes:

    --validate    CPU-only structural/geometric validation of the export
                  (works without Isaac Sim installed and is covered by
                  tools/isaac/tests).
    (default)     rendering playback; requires a supported GPU and NVIDIA's
                  Isaac Sim Python environment.

STATUS: the rendering path is NOT yet validated on a GPU workstation; it
must pass the stage 1 Isaac smoke test before being claimed working.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

from conversion import ConversionError, scene_pose

SCHEMA = "fleet-isaac-replay/1"


class ReplayDataError(ValueError):
    """The export directory does not satisfy the fleet-isaac-replay/1 contract."""


def _parse_tick(value, label: str) -> int:
    # 64-bit ticks are decimal strings in the export (ADR-020).
    if isinstance(value, bool) or not isinstance(value, (int, str)):
        raise ReplayDataError(f"{label} must be a decimal-string tick, got {value!r}")
    try:
        return int(str(value), 10)
    except ValueError as error:
        raise ReplayDataError(f"{label} is not a decimal tick: {value!r}") from error


def _require_finite_fields(record: dict, fields, label: str) -> None:
    for field in fields:
        if field not in record:
            raise ReplayDataError(f"{label} is missing field '{field}'")
        value = record[field]
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ReplayDataError(f"{label}.{field} is not numeric: {value!r}")
        if not math.isfinite(float(value)):
            raise ReplayDataError(f"{label}.{field} is not finite: {value!r}")


def load_export(export_dir) -> dict:
    """Loads and fully validates one export directory (no Isaac required)."""
    export_path = Path(export_dir)
    manifest_path = export_path / "manifest.json"
    poses_path = export_path / "poses.jsonl"
    try:
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise ReplayDataError(f"cannot read manifest: {error}") from error

    if manifest.get("schema") != SCHEMA:
        raise ReplayDataError(f"unsupported schema {manifest.get('schema')!r}")
    _require_finite_fields(manifest, ["footprint_radius_m"], "manifest")
    if float(manifest["footprint_radius_m"]) <= 0.0:
        raise ReplayDataError("manifest.footprint_radius_m must be positive")
    _require_finite_fields(manifest.get("origin", {}),
                           ["latitude_deg", "longitude_deg"], "manifest.origin")
    for field in ("first_tick_ms", "final_tick_ms", "snapshot_period_ms"):
        manifest[field] = _parse_tick(manifest.get(field), f"manifest.{field}")

    declared_robots = manifest.get("robots")
    if not isinstance(declared_robots, list) or not declared_robots:
        raise ReplayDataError("manifest must declare a non-empty robots list")
    declared_ids = [robot.get("robot_id") for robot in declared_robots]
    if any(not isinstance(robot_id, int) or isinstance(robot_id, bool)
           for robot_id in declared_ids):
        raise ReplayDataError(f"manifest robot ids must be integers: {declared_ids!r}")
    if declared_ids != sorted(declared_ids) or len(set(declared_ids)) != len(declared_ids):
        raise ReplayDataError("manifest robots must be unique and ascending by robot_id")

    try:
        lines = poses_path.read_text(encoding="utf-8").splitlines()
    except OSError as error:
        raise ReplayDataError(f"cannot read poses: {error}") from error
    snapshots = []
    previous_tick = None
    for number, line in enumerate(lines, start=1):
        if not line.strip():
            continue
        try:
            snapshot = json.loads(line)
        except json.JSONDecodeError as error:
            raise ReplayDataError(f"poses line {number} is not JSON: {error}") from error
        if snapshot.get("schema") != SCHEMA:
            raise ReplayDataError(f"poses line {number} has wrong schema")
        tick = _parse_tick(snapshot.get("tick_ms"), f"poses line {number} tick_ms")
        if previous_tick is not None and tick <= previous_tick:
            raise ReplayDataError(f"poses line {number} does not advance strictly past "
                                  f"{previous_tick}")
        previous_tick = tick

        entries = snapshot.get("robots")
        if not isinstance(entries, list) or len(entries) != len(declared_robots):
            raise ReplayDataError(f"poses line {number} must carry every declared robot "
                                  "exactly once")
        entry_ids = []
        for entry in entries:
            entry_ids.append(entry.get("robot_id"))
            label = f"poses line {number} robot {entry.get('robot_id')!r}"
            _require_finite_fields(entry.get("truth", {}),
                                   ["latitude_deg", "longitude_deg", "heading_rad"],
                                   f"{label} truth")
            belief = entry.get("belief")
            if belief is not None:
                _require_finite_fields(
                    belief, ["latitude_deg", "longitude_deg", "heading_rad"],
                    f"{label} belief")
                # A present belief must also carry its represented timestamp.
                _parse_tick(belief.get("estimated_at_ms"),
                            f"{label} belief.estimated_at_ms")
        if entry_ids != declared_ids:
            raise ReplayDataError(f"poses line {number} robots {entry_ids!r} do not match "
                                  f"the manifest order {declared_ids!r}")
        snapshots.append(snapshot)

    if not snapshots:
        raise ReplayDataError("poses.jsonl contains no snapshots")
    if _parse_tick(snapshots[0].get("tick_ms"), "first tick") != manifest["first_tick_ms"]:
        raise ReplayDataError("first snapshot does not match manifest.first_tick_ms")
    if _parse_tick(snapshots[-1].get("tick_ms"), "last tick") != manifest["final_tick_ms"]:
        raise ReplayDataError("last snapshot does not match manifest.final_tick_ms")
    return {"manifest": manifest, "snapshots": snapshots, "path": export_path}


def validate_export(export_dir) -> dict:
    """CPU-only validation: structure plus ENU conversion of every pose."""
    data = load_export(export_dir)
    manifest = data["manifest"]
    origin = manifest["origin"]
    footprint = float(manifest["footprint_radius_m"])
    converted = 0
    for snapshot in data["snapshots"]:
        for entry in snapshot["robots"]:
            # Both channels must convert; a null belief has no marker (ADR-020).
            scene_pose(entry["truth"], entry["truth"]["heading_rad"], origin, footprint)
            converted += 1
            if entry.get("belief") is not None:
                scene_pose(entry["belief"], entry["belief"]["heading_rad"], origin, footprint)
                converted += 1
    return {
        "scenario": manifest.get("scenario_name"),
        "seed": manifest.get("resolved_seed"),
        "snapshots": len(data["snapshots"]),
        "robots": len(manifest["robots"]),
        "poses_converted": converted,
        "final_tick_ms": manifest["final_tick_ms"],
        "observed_extent_m": manifest.get("observed_extent_m"),
    }


def _define_cube(stage, path, size, color):
    from pxr import Gf, UsdGeom

    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(float(size))
    translate = cube.AddTranslateOp()
    orient = cube.AddOrientOp()
    cube.CreateDisplayColorPrimvar().Set([Gf.Vec3f(*color)])
    return translate, orient


def _define_sphere(stage, path, radius, color):
    from pxr import Gf, UsdGeom

    sphere = UsdGeom.Sphere.Define(stage, path)
    sphere.CreateRadiusAttr(float(radius))
    translate = sphere.AddTranslateOp()
    orient = sphere.AddOrientOp()
    sphere.CreateDisplayColorPrimvar().Set([Gf.Vec3f(*color)])
    return translate, orient


def _play(data, app, rate, loop):
    # Imports are Isaac-side and only valid after SimulationApp launch.
    import omni.usd
    from pxr import Gf, UsdGeom

    manifest = data["manifest"]
    origin = manifest["origin"]
    footprint = float(manifest["footprint_radius_m"])

    context = omni.usd.get_context()
    context.new_stage()
    stage = context.get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    # Floor: a flat slab under the whole footprint; replay-only scenery.
    floor_extent = footprint + 5.0
    floor_translate, _ = _define_cube(stage, "/World/Floor", 2.0 * floor_extent,
                                      (0.25, 0.25, 0.28))
    floor_translate.Set(Gf.Vec3d(0.0, 0.0, -0.05))

    # Truth = blue cube; belief = orange sphere raised above it. Distinct
    # geometry and color stand in for labels in this first viewer.
    truth_markers = {}
    belief_markers = {}
    for robot in manifest["robots"]:
        robot_id = robot["robot_id"]
        truth_markers[robot_id] = _define_cube(
            stage, f"/World/Truth/robot_{robot_id}", 0.6, (0.10, 0.30, 0.90))
        belief_markers[robot_id] = _define_sphere(
            stage, f"/World/Belief/robot_{robot_id}", 0.25, (1.00, 0.55, 0.10))

    for _ in range(max(1, loop)):
        previous_tick = None
        for snapshot in data["snapshots"]:
            tick = int(snapshot["tick_ms"])
            if previous_tick is not None and rate > 0.0:
                # Presentation pacing only: derived from RECORDED ticks, and
                # it never feeds anything back into the data or the export.
                time.sleep(max(0.0, (tick - previous_tick) / 1000.0 / rate))
            previous_tick = tick
            for entry in snapshot["robots"]:
                robot_id = entry["robot_id"]
                pose = scene_pose(entry["truth"], entry["truth"]["heading_rad"],
                                  origin, footprint)
                position = pose["position_enu_m"]
                quaternion = pose["orientation_wxyz"]
                translate, orient = truth_markers[robot_id]
                translate.Set(Gf.Vec3d(position["east"], position["north"], 0.3))
                orient.Set(Gf.Quatf(quaternion["w"],
                                    Gf.Vec3f(quaternion["x"], quaternion["y"],
                                             quaternion["z"])))

                translate, orient = belief_markers[robot_id]
                belief = entry.get("belief")
                if belief is None:
                    # No estimate: no belief marker — never a marker at the
                    # origin (ADR-020).
                    translate.Set(Gf.Vec3d(0.0, 0.0, -1000.0))
                    continue
                pose = scene_pose(belief, belief["heading_rad"], origin, footprint)
                position = pose["position_enu_m"]
                quaternion = pose["orientation_wxyz"]
                translate.Set(Gf.Vec3d(position["east"], position["north"], 1.1))
                orient.Set(Gf.Quatf(quaternion["w"],
                                    Gf.Vec3f(quaternion["x"], quaternion["y"],
                                             quaternion["z"])))
            app.update()


def run_viewer(export_dir, headless, rate, loop):
    """Rendering playback. Requires NVIDIA Isaac Sim's Python on a GPU box."""
    data = load_export(export_dir)
    try:
        from isaacsim import SimulationApp
    except ImportError as error:
        raise SystemExit(
            "replay: rendering requires NVIDIA Isaac Sim's Python environment; "
            "use --validate for the CPU-only checks"
        ) from error

    app = SimulationApp({"headless": headless})
    try:
        _play(data, app, rate, loop)
    finally:
        # Guaranteed cleanup even if playback raises (docs/isaac contract).
        app.close()


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description="Isaac Sim stage 1 read-only replay viewer (ADR-020)")
    parser.add_argument("--export-dir", required=True,
                        help="directory holding manifest.json + poses.jsonl")
    parser.add_argument("--validate", action="store_true",
                        help="CPU-only validation; no Isaac required")
    parser.add_argument("--headless", action="store_true",
                        help="run Isaac without a window (smoke mode)")
    parser.add_argument("--rate", type=float, default=1.0,
                        help="playback rate multiplier (default 1.0 = recorded pace)")
    parser.add_argument("--loop", type=int, default=1,
                        help="number of playback passes (default 1)")
    arguments = parser.parse_args(argv)

    try:
        if arguments.validate:
            summary = validate_export(arguments.export_dir)
            print("replay: export valid:")
            for key, value in summary.items():
                print(f"  {key}: {value}")
            return 0
        run_viewer(arguments.export_dir, arguments.headless, arguments.rate, arguments.loop)
        return 0
    except (ReplayDataError, ConversionError) as error:
        print(f"replay: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
