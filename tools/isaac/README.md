# Isaac stage 1 bridge tools (read-only replay, ADR-020)

Status: stage 1 implemented; the CPU-only export and conversion layers are
tested in CI-able runs. **The Isaac rendering path has not yet been executed
on a supported GPU workstation** — it must pass the workstation smoke test
(docs/isaac/SETUP.md) before the bridge is claimed working end to end.

## Pieces

```text
conversion.py        pure WGS84 <-> scene-frame math (ENU, yaw, quaternion);
                     importable WITHOUT Isaac; the single georeferencing seam
replay.py            export loader/validator (CPU) + Isaac viewer (GPU)
tests/               stdlib unittest suites; no Isaac import anywhere
```

## Export a run (CPU, no Isaac needed)

Build with the opt-in tools, then export:

```sh
cmake --preset debug -DFLEET_WERROR=ON -DFLEET_BUILD_ISAAC_TOOLS=ON
cmake --build --preset debug
./build/debug/apps/fleet_isaac_export/fleet_isaac_export \
    --scenario scenarios/isaac_replay.json \
    --out build/isaac-export \
    --seed 17 \
    --trace build/isaac-export/trace.jsonl
```

`scenarios/isaac_replay.json` runs against the exporter's own bounded L
fixture map (nodes A, B, C within ~30 m of origin 52.370/9.730): a straight
leg, a corner, a second leg and a stop, with perfect GNSS so belief markers
exist. Note: running the same file under `fleet_sim` uses fleet_sim's large
demo grid instead (node names coincide) — only `fleet_isaac_export` supplies
the Isaac fixture map.

The export is observation-only: its trace is byte-identical to a plain
`ScenarioRunner` run of the same scenario and seed on the same map (locked
by tests), and two exports of the same inputs are byte-identical.

## Validate an export (CPU)

```sh
python3 tools/isaac/replay.py --validate --export-dir build/isaac-export
```

This checks the full `fleet-isaac-replay/1` contract: schema, decimal-string
ticks, strictly advancing time, complete ascending robot sets per frame,
finite named fields, and ENU conversion of every pose inside the manifest
footprint.

## View in Isaac Sim (GPU workstation)

Run through NVIDIA's bundled Python (see docs/isaac/SETUP.md for the pinned
Isaac Sim version; example path — adjust to your install):

```sh
<isaac-sim-root>/python.sh tools/isaac/replay.py --export-dir build/isaac-export
```

Truth is the blue cube; belief is the orange sphere above it (no sphere
before the first fix). Playback paces by recorded ticks, never by render
FPS; `--rate` rescales presentation only. Reset = replay: run the command
again (or `--loop N`). This path is the unvalidated-on-GPU part.

## Python tests (CPU)

```sh
python3 -m unittest discover -s tools/isaac/tests -v
```

No Isaac import occurs anywhere in the test tree; the viewer's Isaac-side
imports are lazy and only execute in rendering mode.
