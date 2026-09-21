"""Pure-Python tests for the stage 1 conversion contract (ADR-020).

Expected values are derived independently from the documented contract, not
from round-tripping the implementation: cardinal quaternions use exact
literals, ENU checks verify scale/sign/wrap semantics against hand-set
constants.
"""

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from conversion import (
    ConversionError,
    enu_from_wgs84,
    normalize_heading_rad,
    quaternion_wxyz_from_heading_rad,
    scene_pose,
    wgs84_from_enu,
    wrap_longitude_deg,
    yaw_from_heading_rad,
)

ORIGIN = {"latitude_deg": 52.370, "longitude_deg": 9.730}
SQRT_HALF = math.sqrt(0.5)


class WrapLongitudeTest(unittest.TestCase):
    def test_wraps_positive(self):
        self.assertAlmostEqual(wrap_longitude_deg(190.0), -170.0)

    def test_wraps_negative(self):
        self.assertAlmostEqual(wrap_longitude_deg(-190.0), 170.0)

    def test_antimeridian_shortest(self):
        # +0.0002 deg across the antimeridian is a small EAST step.
        self.assertAlmostEqual(wrap_longitude_deg(-179.9999 - 179.9999), 0.0002)


class EnuFromWgs84Test(unittest.TestCase):
    def test_origin_maps_exactly_to_zero(self):
        horizontal = enu_from_wgs84(52.370, 9.730, ORIGIN, 50.0)
        self.assertEqual(horizontal["east_m"], 0.0)
        self.assertEqual(horizontal["north_m"], 0.0)

    def test_north_scale_matches_pinned_radius(self):
        # 0.001 deg latitude at R = 6371008.8 m: 111.19508023406387 m
        # (independent hand derivation of the pinned contract constant).
        horizontal = enu_from_wgs84(52.371, 9.730, ORIGIN, 200.0)
        self.assertAlmostEqual(horizontal["north_m"], 111.19508023406387, delta=1e-9)
        self.assertEqual(horizontal["east_m"], 0.0)

    def test_east_uses_anchor_latitude_cosine_and_is_positive(self):
        # 0.001 deg longitude at anchor 52.370 deg shrinks by cos(52.370 deg).
        horizontal = enu_from_wgs84(52.370, 9.731, ORIGIN, 200.0)
        expected = 0.001 * (6371008.8 * math.pi / 180.0) * math.cos(math.radians(52.370))
        self.assertAlmostEqual(horizontal["east_m"], expected, delta=1e-9)
        self.assertEqual(horizontal["north_m"], 0.0)

    def test_south_and_west_are_negative(self):
        horizontal = enu_from_wgs84(52.3699, 9.7299, ORIGIN, 50.0)
        self.assertLess(horizontal["north_m"], 0.0)
        self.assertLess(horizontal["east_m"], 0.0)

    def test_antimeridian_origin_wraps(self):
        origin = {"latitude_deg": 52.370, "longitude_deg": 179.9999}
        horizontal = enu_from_wgs84(52.370, -179.9999, origin, 200.0)
        self.assertGreater(horizontal["east_m"], 0.0)

    def test_outside_footprint_rejected(self):
        with self.assertRaises(ConversionError):
            enu_from_wgs84(52.372, 9.730, ORIGIN, 50.0)  # ~222 m north

    def test_polar_anchor_rejected(self):
        origin = {"latitude_deg": 81.0, "longitude_deg": 9.730}
        with self.assertRaises(ConversionError):
            enu_from_wgs84(81.0, 9.730, origin, 50.0)

    def test_non_finite_rejected(self):
        with self.assertRaises(ConversionError):
            enu_from_wgs84(float("nan"), 9.730, ORIGIN, 50.0)

    def test_inverse_uses_same_fixed_origin_and_scale(self):
        # Round trip is an additional check, not the primary evidence.
        horizontal = enu_from_wgs84(52.3702, 9.7303, ORIGIN, 50.0)
        back = wgs84_from_enu(horizontal["east_m"], horizontal["north_m"], ORIGIN)
        self.assertAlmostEqual(back["latitude_deg"], 52.3702, delta=1e-12)
        self.assertAlmostEqual(back["longitude_deg"], 9.7303, delta=1e-12)


class HeadingConversionTest(unittest.TestCase):
    def test_normalize_range_and_negative(self):
        self.assertEqual(normalize_heading_rad(-0.5), 2.0 * math.pi - 0.5)
        self.assertEqual(normalize_heading_rad(2.0 * math.pi + 0.25), 0.25)

    def test_yaw_cardinals_match_architecture_table(self):
        # docs/isaac/ARCHITECTURE.md: East 0, North pi/2, West pi, South -pi/2.
        self.assertAlmostEqual(yaw_from_heading_rad(math.pi / 2), 0.0)
        self.assertAlmostEqual(yaw_from_heading_rad(0.0), math.pi / 2)
        self.assertAlmostEqual(yaw_from_heading_rad(math.pi), -math.pi / 2)
        self.assertAlmostEqual(abs(yaw_from_heading_rad(3.0 * math.pi / 2)), math.pi)

    def test_quaternion_identity_when_facing_east(self):
        quaternion = quaternion_wxyz_from_heading_rad(math.pi / 2)
        self.assertAlmostEqual(quaternion["w"], 1.0, delta=1e-12)
        self.assertAlmostEqual(quaternion["x"], 0.0, delta=1e-12)
        self.assertAlmostEqual(quaternion["y"], 0.0, delta=1e-12)
        self.assertAlmostEqual(quaternion["z"], 0.0, delta=1e-12)

    def test_quaternion_north_is_quarter_turn_about_plus_z(self):
        quaternion = quaternion_wxyz_from_heading_rad(0.0)
        self.assertAlmostEqual(quaternion["w"], SQRT_HALF, delta=1e-12)
        self.assertAlmostEqual(quaternion["x"], 0.0, delta=1e-12)
        self.assertAlmostEqual(quaternion["y"], 0.0, delta=1e-12)
        self.assertAlmostEqual(quaternion["z"], SQRT_HALF, delta=1e-12)

    def test_quaternion_west_is_quarter_turn_about_minus_z(self):
        quaternion = quaternion_wxyz_from_heading_rad(math.pi)
        self.assertAlmostEqual(quaternion["w"], SQRT_HALF, delta=1e-12)
        self.assertAlmostEqual(quaternion["z"], -SQRT_HALF, delta=1e-12)

    def test_quaternion_south_is_half_turn(self):
        quaternion = quaternion_wxyz_from_heading_rad(3.0 * math.pi / 2)
        self.assertAlmostEqual(quaternion["w"], 0.0, delta=1e-12)
        self.assertAlmostEqual(abs(quaternion["z"]), 1.0, delta=1e-12)

    def test_quaternion_is_unit_and_planar(self):
        for heading in (0.0, 0.7, 1.9, 3.3, 5.8):
            quaternion = quaternion_wxyz_from_heading_rad(heading)
            norm = math.sqrt(sum(component * component
                                 for component in quaternion.values()))
            self.assertAlmostEqual(norm, 1.0, delta=1e-12)
            self.assertEqual(quaternion["x"], 0.0)
            self.assertEqual(quaternion["y"], 0.0)

    def test_non_finite_heading_rejected(self):
        with self.assertRaises(ConversionError):
            quaternion_wxyz_from_heading_rad(float("inf"))

    def test_scene_pose_shape_and_footprint(self):
        pose = scene_pose({"latitude_deg": 52.3702, "longitude_deg": 9.7303},
                          1.5708, ORIGIN, 50.0)
        self.assertEqual(set(pose), {"position_enu_m", "orientation_wxyz"})
        self.assertEqual(set(pose["position_enu_m"]), {"east", "north", "up"})
        self.assertEqual(pose["position_enu_m"]["up"], 0.0)
        self.assertGreater(pose["position_enu_m"]["north"], 20.0)
        with self.assertRaises(ConversionError):
            scene_pose({"latitude_deg": 52.372, "longitude_deg": 9.730}, 0.0, ORIGIN, 50.0)


if __name__ == "__main__":
    unittest.main()
