"""Pure WGS84 <-> scene-frame conversions for the Isaac stage 1 replay bridge.

ADR-020 (docs/design_decisions/ADR-020-isaac-stage1-readonly-replay-export.md)
places the georeferencing seam HERE and only here: the native exporter emits
named WGS84 fields plus a pinned origin, and this module maps them into the
Isaac scene frame. It is deliberately importable WITHOUT Isaac Sim installed.

Contract (docs/isaac/ARCHITECTURE.md "Coordinate and unit contract"):

    R = 6371008.8 m
    meters_per_degree = R * pi / 180
    east  = wrapped longitude difference * meters_per_degree * cos(origin_lat)
    north = latitude difference * meters_per_degree
    axes: X east, Y north, Z up; one stage unit = one meter
    heading: radians clockwise from north (domain convention)
    yaw (counterclockwise from world +X) = pi/2 - heading
    quaternions: scalar-first (w, x, y, z), about +Z only (planar fixture)

This is a bounded engineering frame for the ~50 m fixture, not a general
projection: it rejects anchor latitudes beyond +/-80 degrees and positions
outside the manifest's footprint radius rather than silently clamping.
"""

from __future__ import annotations

import math

EARTH_RADIUS_M = 6371008.8
METERS_PER_DEGREE = EARTH_RADIUS_M * math.pi / 180.0
MAX_ABS_ANCHOR_LATITUDE_DEG = 80.0

# Sub-micrometer slack so a pose exactly on the footprint boundary is not
# rejected by floating-point rounding.
_FOOTPRINT_SLACK_M = 1e-6
_TWO_PI = 2.0 * math.pi


class ConversionError(ValueError):
    """A value outside the bounded adapter contract was rejected."""


def _require_finite(label: str, *values: float) -> None:
    for value in values:
        if not math.isfinite(value):
            raise ConversionError(f"non-finite {label} rejected by the conversion layer")


def wrap_longitude_deg(delta_deg: float) -> float:
    """Wraps a longitude difference into (-180, +180]."""
    wrapped = math.fmod(delta_deg + 180.0, 360.0)
    if wrapped <= 0.0:
        wrapped += 360.0
    return wrapped - 180.0


def _validate_origin(origin: dict) -> tuple[float, float]:
    try:
        anchor_lat = float(origin["latitude_deg"])
        anchor_lon = float(origin["longitude_deg"])
    except (KeyError, TypeError) as error:
        raise ConversionError("origin must carry latitude_deg/longitude_deg") from error
    _require_finite("origin", anchor_lat, anchor_lon)
    if abs(anchor_lat) > MAX_ABS_ANCHOR_LATITUDE_DEG:
        raise ConversionError(
            f"anchor latitude {anchor_lat} deg is beyond the supported +/-80 deg domain"
        )
    return anchor_lat, anchor_lon


def enu_from_wgs84(
    latitude_deg: float, longitude_deg: float, origin: dict, footprint_radius_m: float
) -> dict:
    """Maps one WGS84 position to east/north meters around the fixed origin."""
    _require_finite("position", latitude_deg, longitude_deg, footprint_radius_m)
    if not -90.0 <= latitude_deg <= 90.0 or not -180.0 <= longitude_deg <= 180.0:
        raise ConversionError(
            f"position ({latitude_deg}, {longitude_deg}) is outside WGS84 bounds"
        )
    anchor_lat, anchor_lon = _validate_origin(origin)

    east = (
        wrap_longitude_deg(longitude_deg - anchor_lon)
        * METERS_PER_DEGREE
        * math.cos(math.radians(anchor_lat))
    )
    north = (latitude_deg - anchor_lat) * METERS_PER_DEGREE

    if math.hypot(east, north) > footprint_radius_m + _FOOTPRINT_SLACK_M:
        raise ConversionError(
            f"position ({latitude_deg}, {longitude_deg}) lies outside the "
            f"{footprint_radius_m} m replay footprint"
        )
    return {"east_m": east, "north_m": north}


def wgs84_from_enu(east_m: float, north_m: float, origin: dict) -> dict:
    """Inverse of enu_from_wgs84 with the same fixed origin and scale."""
    _require_finite("displacement", east_m, north_m)
    anchor_lat, anchor_lon = _validate_origin(origin)
    latitude = anchor_lat + north_m / METERS_PER_DEGREE
    if not -90.0 <= latitude <= 90.0:
        raise ConversionError(f"north displacement {north_m} m would cross a pole")
    longitude = wrap_longitude_deg(
        anchor_lon + east_m / (METERS_PER_DEGREE * math.cos(math.radians(anchor_lat)))
    )
    return {"latitude_deg": latitude, "longitude_deg": longitude}


def normalize_heading_rad(heading_rad: float) -> float:
    """Normalizes a heading to [0, 2*pi); rejects non-finite values."""
    _require_finite("heading", heading_rad)
    normalized = math.fmod(heading_rad, _TWO_PI)
    if normalized < 0.0:
        normalized += _TWO_PI
    return normalized


def yaw_from_heading_rad(heading_rad: float) -> float:
    """Isaac yaw (CCW from world +X) for a heading clockwise from north."""
    yaw = math.pi / 2.0 - normalize_heading_rad(heading_rad)
    # Normalize into (-pi, pi]; -pi and pi are the same facing, keep +pi.
    while yaw > math.pi:
        yaw -= _TWO_PI
    while yaw <= -math.pi:
        yaw += _TWO_PI
    return yaw


def quaternion_wxyz_from_heading_rad(heading_rad: float) -> dict:
    """Planar yaw about +Z as a scalar-first (w, x, y, z) unit quaternion.

    Component order is pinned deliberately: USD/Core interfaces are
    scalar-first, some lower-level interfaces are (x, y, z, w). Callers must
    consume this dict by name, never by position.
    """
    half_yaw = yaw_from_heading_rad(heading_rad) / 2.0
    quaternion = {
        "w": math.cos(half_yaw),
        "x": 0.0,
        "y": 0.0,
        "z": math.sin(half_yaw),
    }
    _require_finite("quaternion", *quaternion.values())
    return quaternion


def scene_pose(position_deg: dict, heading_rad: float, origin: dict,
               footprint_radius_m: float) -> dict:
    """Composite Isaac scene pose for one exported truth/belief record."""
    try:
        latitude = float(position_deg["latitude_deg"])
        longitude = float(position_deg["longitude_deg"])
    except (KeyError, TypeError) as error:
        raise ConversionError(
            "position must carry named latitude_deg/longitude_deg fields"
        ) from error
    horizontal = enu_from_wgs84(latitude, longitude, origin, footprint_radius_m)
    return {
        "position_enu_m": {
            "east": horizontal["east_m"],
            "north": horizontal["north_m"],
            "up": 0.0,  # planar fixture: altitude is presentation-only (ADR-020)
        },
        "orientation_wxyz": quaternion_wxyz_from_heading_rad(heading_rad),
    }
