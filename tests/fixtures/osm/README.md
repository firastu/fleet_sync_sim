# OSM test fixtures

- `tiny_network.osm.pbf` — main import fixture (a tiny road network with
  a deliberately bent bidirectional way — its summed-polyline cost must
  exceed the endpoint geodesic — plus `oneway=yes`, `oneway=-1`, an
  excluded footway and intermediate geometry nodes).
  `tiny_network.osm` is its human-readable mirror for Git review.
- `tiny_invalid_oneway.osm.pbf` — `oneway=alternating` (outside the
  supported subset; the import must fail).
- `tiny_antiparallel.osm.pbf` — two ways connecting the same node pair
  in opposite directions (the import must fail per ADR-013).
- `tiny_parallel_same_direction.osm.pbf` — two ways connecting the same
  node pair in the SAME direction (still two physical roads; must fail
  exactly like the anti-parallel case).
- `tiny_roundabout.osm.pbf` — `junction=roundabout` without an explicit
  `oneway` tag (implicit direction unsupported; must fail loudly, never
  import as bidirectional).
- `tiny_self_loop_pair.osm.pbf` — one way whose refs revisit the same
  retained node pair (segments 60->61 and 61->60 from a single way;
  must fail with both segment indices named).

The `.osm.pbf` artifacts are committed so CI never needs OSM tooling.
The source of truth for their content is the embedded data in
`generate_fixture.cpp`. Regenerate on demand with:

    cmake --build build/debug --target fleet_osm_fixture_writer
    rm -f tests/fixtures/osm/*.osm.pbf   # the writer refuses to overwrite
    ./build/debug/tests/fleet_osm_fixture_writer tests/fixtures/osm

Regeneration changes binary bytes (PBF header timestamps) but never the
logical content the importer consumes.
