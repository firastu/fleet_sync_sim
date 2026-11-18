# ADR-022: Terrain elevation grids and SRTM import

* Status: Proposed (implemented; awaiting #19 review)
* Date: 2026-11-18
* Scope: M4 #19; `fleet::map::ElevationGrid`, the `fleet_dem_import` target
  (`fleet::dem`), the terrain roles and the metric-frame rule for the terrain
  domain. No motion, sensing, scenario, trace or console change.

## Context

M4 studies navigation without GNSS by matching sensed terrain height against a
locally held elevation map. Every later step (terrain motion #20, altitude
sensors #21, the estimator #22) needs one shared answer to two questions: what
is the terrain height at this WGS84 position, and what happens where the map
has no answer? The answer must be deterministic, must never invent heights,
and must keep simulation truth separate from what a robot carries (ADR-011).

## Decision

### `ElevationGrid`: an immutable lattice of point samples

`fleet::map::ElevationGrid` holds heights in meters on a regular WGS84
latitude/longitude lattice. It lives in the map core beside `MapGeometry`,
because world truth and robots both hold terrain as map data; it is
independent of `Graph` and `BaseMap`.

* The layout stores CORNERS (the north-west and south-east sample positions)
  plus row and column counts; spacing is derived. Samples are points, not
  cell areas. Corners as the stored truth keep lattice edges exact, so
  adjacent SRTM tiles share their edge coordinates bit for bit; a stored
  spacing such as 1/3600 degree is not representable and would drift at the
  far edge.
* North > south and east > west: a grid never crosses the antimeridian. At
  least 2 x 2 samples.
* Samples are row-major, north row first, west column first, stored as
  `float`: SRTM's 16-bit integers convert exactly, and a 3601 x 3601 tile
  takes 52 MB instead of 104 MB as `double`. NaN marks NODATA; infinities
  are rejected. NaN never escapes the type: lookups report NODATA as a
  status.

### Lookups never invent heights

`height_at(position)` returns `ElevationLookup{status, height_m}`;
`height_m` is engaged exactly when `status == Ok`.

* `OutsideCoverage`: outside the closed box between the outermost samples.
  Never clamped to the edge, never extrapolated.
* `NoData`: a sample the interpolation needs is a void.
* `Ok`: bilinear interpolation of the enclosing cell. A sample whose weight
  is exactly zero is not consulted, so a lookup exactly on a sample returns
  that sample, and a lookup on a cell edge uses only that edge's two samples.
  A void therefore does not affect lookups on neighbouring samples or edges.
* Lattice coordinates are `(north - lat) / (north - south) * (rows - 1)` (and
  the column equivalent). IEEE rounding is monotonic, so every position
  inside the closed box maps into the lattice without clamping, and the
  edges map exactly to the first and last rows and columns.
* An invalid query position (non-finite, outside WGS84 bounds) is a caller
  bug: `std::invalid_argument`, not a coverage status.
* A lookup is a pure function of (grid, position) with a fixed summation
  order. It consumes no randomness.

Whether `OutsideCoverage` or `NoData` becomes a sensor dropout, a scenario
error or an estimator input is decided by the consumer (#20-#22), never by
substituting a value here.

### What the heights mean

* `ElevationSurface` is mandatory metadata: `SurfaceModel` (canopy and roofs,
  what a downward rangefinder sees) or `BareEarth` (the ground a ground robot
  drives on). The two differ by vegetation and building height, and
  consumers must be able to tell which one they hold.
* `VerticalDatum` labels the reference of the heights. FleetSyncSim never
  converts between datums: it keeps source heights and makes the reference
  explicit, so grids on different references are never mixed silently. One
  value exists (`Egm96`, SRTM's); others are added together with the source
  that needs them.

### SRTM `.hgt` import: `fleet::dem`

A separate target, `fleet_dem_import` (namespace `fleet::dem`), mirrors
`fleet_osm_import` (ADR-014): the map core knows no file formats. The format
needs no third-party dependency.

* Tile identity comes from the file name: `<tile>.hgt`, where the tile name
  is `[NS]dd[EW]ddd` in uppercase and names the integer south-west corner.
  `S00`, `W000` and out-of-range names are rejected. A renamed file cannot
  silently land somewhere else.
* Size decides resolution: exactly 3601 x 3601 or 1201 x 1201 big-endian
  signed 16-bit samples. Any other size is rejected before allocation; there
  is no guessing.
* -32768 is a void and becomes NODATA.
* Sample (0, 0) lies on (south + 1, west); the last sample on
  (south, west + 1).
* Heights are EGM96, the format's datum. The caller states the
  `ElevationSurface`: SRTM itself is a surface model, but other producers
  ship bare-earth data in the same container.
* Errors name the offending file or tile name.
* One tile per grid: mosaicking adjacent tiles is not part of #19.

### Two roles: truth terrain and robot terrain map

World truth terrain and each robot's terrain map are separate handles,
introduced when a scenario first loads terrain (#20). Robots hold their own
map handle; truth terrain stays in `fleet::world` and reaches robots only
through sensor models (#21), per ADR-011. Grids are immutable, so both roles
may point at the same grid when a scenario declares the robot's map perfect.
A robot map that differs from truth (coarser, biased, older) is a distinct
grid, never a mutation. `ElevationGrid` itself does not know which role it
plays.

### Metric frame: no scenario-global projection

Terrain-domain positions stay WGS84 everywhere: in truth, in sensor outputs
and in robot belief. Motion and dead reckoning advance positions by small
metric displacements through the existing single metric seam,
`localization::apply_en_displacement` (ADR-017), anchored at the current
position on every step. Its validity domain then bounds the STEP (meters to
tens of meters), not the scenario extent, so kilometer-scale motion needs no
new projection and no maximum-extent rule. #20 fixes the step bound. An
estimator may use an internal local frame as an implementation detail
recorded in its own ADR (#22); such a frame is never a shared contract.

## Alternatives rejected

* **GDAL / GeoTIFF first.** Copernicus GLO-30 and most national DEMs are
  GeoTIFF, but GDAL is a large GIS dependency (AGENTS.md section 6), and the
  first useful global source, SRTM, needs none. A GeoTIFF reader can follow,
  `PRIVATE` to the importer, once a concrete source justifies it.
* **Stored spacing instead of corners.** Non-representable spacings
  (1/3600 degree) make the far edge drift by ulps, and adjacent tiles would
  disagree about their shared edge.
* **`int16` storage.** Exact for SRTM, but cannot represent fractional
  synthetic or resampled grids. **`double` storage:** twice the memory for no
  source precision.
* **Clamping or extrapolating at the edge, zero-filling voids.** Each
  produces a plausible wrong height, the silent-fallback class ADR-014
  rejects. Explicit statuses keep the failure visible.
* **Nearest-neighbour or bicubic interpolation.** Nearest-neighbour creates
  step discontinuities that look like false terrain features; bicubic needs
  a 4 x 4 neighbourhood (a larger void footprint) and can overshoot.
  Bilinear is continuous, local and exact on samples.
* **A sentinel height (such as -32768) in the public API.** A sentinel is a
  height that is not a height. NaN stays internal; the API reports a status.
* **A scenario-anchored ENU tangent plane, or UTM.** Both add a second metric
  seam: the tangent plane distorts more with extent and needs a maximum-extent
  rule; UTM brings zone boundaries and long series formulas. Per-step
  displacement through the existing seam avoids both.

## Consequences and limitations

* #19 changes no motion, sensor, scenario, trace or console behavior. Every
  existing trace, including the founding trace, is unchanged.
* One tile per grid: an area spanning tile edges needs mosaicking, a later
  explicit addition.
* A grid touching +180 degrees does not answer queries at -180 (and vice
  versa). Terrain scenarios near the antimeridian or the poles are
  unsupported, matching `apply_en_displacement`'s domain.
* Public DEMs are coarse (SRTM: about 30 m at 1 arc-second, 90 m at
  3 arc-seconds) and are surface models; bare-earth data must come from other
  sources and be labelled as such.
* The vertical datum is a label only. Comparing a GNSS ellipsoidal height
  with an EGM96 grid would need a conversion that does not exist yet.
* Full DEM tiles are never committed (`*.hgt` is ignored); tests synthesize
  tiles in memory and in the build tree.

Validation: `tests/unit/map/elevation_grid_test.cpp` and
`tests/unit/dem/hgt_reader_test.cpp` pin the lookup and import contracts:
exact samples, bilinear interiors and edges, closed-box coverage one ulp
beyond each edge, voids with the zero-weight rule, hemisphere signs,
antimeridian non-wrapping, invalid layouts and queries, the tile-name
grammar, both SRTM sizes, big-endian and negative decoding, shared tile
edges, on-disk reading and named errors.
