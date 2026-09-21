# Isaac Workstation Setup

Status: development-tooling instructions, not runtime integration approval.
Start with the [integration overview](README.md) and its milestone gate.

Official documentation checked on 2026-09-21. The proposed initial baseline is
**Isaac Sim 6.1.0, Linux x86_64, workstation archive**. NVIDIA's documentation
currently identifies 6.1 as stable and marks 5.1 unsupported. This is a
documentation baseline, not a claim that this repository has been GPU-tested
with 6.1.0. Pin the actual installation and record the smoke-test result.

## 1. Check the machine before installing

Run these read-only checks in a regular Linux terminal:

```sh
uname -m
cat /etc/os-release
nvidia-smi
free -h
df -h "$HOME"
```

Compare the results with the versioned
[NVIDIA requirements](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/installation/requirements.html).
That page lists Ubuntu 22.04/24.04, a minimum tier with 32 GB RAM, 50 GB SSD,
an RTX 4080-class GPU and 16 GB VRAM; 64 GB RAM and more SSD space are sensible
for development assets and caches. NVIDIA lists Linux driver 595.58.03 as
tested for this release, not as a universal minimum for every GPU.

Use the current driver guidance linked from that page and the compatibility
checker rather than assuming any newer or older driver is valid. A100/H100
compute GPUs without RT cores are not supported by this renderer. A container
or headless flag does not remove GPU requirements.

If `nvidia-smi` fails, stop and ask the workstation administrator to repair the
driver. Do not replace kernel drivers, disable host security settings, or
change system-wide ASLR as part of an application setup recipe.

No suitable GPU? Continue with native unit tests, protocol fixtures and
recorded-input replay. Schedule the physical smoke tests on a compatible
workstation; do not report them as passed locally.

## 2. Establish the native baseline

Use a clean terminal that has not sourced ROS or Isaac environment scripts.
From the FleetSyncSim repository root:

```sh
export FLEET_ROOT="$PWD"
git status --short
git rev-parse HEAD
cmake --preset debug -DFLEET_WERROR=ON
cmake --build --preset debug
ctest --preset debug --output-on-failure
mkdir -p "$FLEET_ROOT/build/isaac-validation"
./build/debug/apps/fleet_sim/fleet_sim \
    --scenario scenarios/station_partition.json --seed 1234 \
    --trace build/isaac-validation/reference-before.jsonl
```

Expected: successful configure/build/test exit codes and a structured trace.
Record the commit and whether the worktree contains changes. Do not discard
someone else's work to obtain a clean tree. For a rigorous before/after
comparison, use the same map, scenario, resolved seed, compiler and platform.

The full native review gates remain debug, ASan+UBSan and TSan; see the
[repository build instructions](../../README.md#building) and
[engineering rules](../../AGENTS.md). Isaac installation must not be necessary
to configure, build or execute those native tests.

## 3. Install the isolated Isaac distribution

Follow NVIDIA's versioned
[workstation guide](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/installation/install_workstation.html)
and [download page](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/installation/download.html).
Download the Linux x86_64 **6.1.0** archive using the official site. Review and
accept the applicable terms yourself; this guide does not accept licenses or
telemetry settings on your behalf.

Verify the downloaded archive against the checksum on the versioned download
page before extracting it. Use NVIDIA's published checksum algorithm for that
artifact, then record a local SHA-256 digest in the experiment manifest. A
vendor MD5 checksum checks accidental corruption; it is not an authenticity
guarantee. Download only from the official source.

In a second terminal, after downloading the archive:

```sh
export ISAAC_ROOT="$HOME/tools/isaac-sim-6.1.0"
mkdir -p "$ISAAC_ROOT"
unzip "$HOME/Downloads/isaac-sim-standalone-6.1.0-linux-x86_64.zip" -d "$ISAAC_ROOT"
cd "$ISAAC_ROOT"
./post_install.sh
./isaac-sim.compatibility_check.sh
```

The archive filename above is the one shown in the versioned NVIDIA guide.
If NVIDIA publishes a different artifact/build, verify its version and update
the recorded environment rather than silently substituting `latest`.
Extract into a new, empty version-specific directory, not over another release.

In the compatibility checker, verify GPU, driver, memory and OS checks, then
run **Test Kit**. Save the report with the experiment artifacts. Red/unsupported
results are a blocker, not a warning to ignore.

Then launch:

```sh
cd "$ISAAC_ROOT"
./isaac-sim.sh
```

Expected: the editor opens and a simple scene renders. Initial shader warm-up
can take several minutes. Capture logs if it fails; do not diagnose FleetSyncSim
until a stock NVIDIA example works. Omniverse Launcher and Nucleus are not
prerequisites for this local workstation workflow.

## 4. Verify standalone stepping

Close the GUI instance before this check if GPU memory is limited. Use the
bundled launcher, not the system Python interpreter or a random virtualenv:

```sh
cd "$ISAAC_ROOT"
./python.sh --no-ros-env \
    standalone_examples/deprecated/api/isaacsim.core.api/time_stepping.py
```

This exact example path is documented by NVIDIA for 6.1.0. It is under
`deprecated`: use it as an installation diagnostic, not as a promise that the
old Core API is the right foundation for a new adapter. Select the supported
6.1 API for new code and record the selected API in the implementation review.
The [implementation checklist](IMPLEMENTATION.md#4-establish-physical-stepping-and-encoder-readings)
identifies the 6.1 stepping/articulation APIs and their experimental-API caveat.
The `--no-ros-env` launcher option avoids automatic ROS environment setup in
this non-ROS experiment.

See the official
[Python lifecycle guide](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/python_scripting/manual_standalone_python.html).
Its important lifecycle rules are:

1. Import and instantiate `SimulationApp` before importing runtime-dependent
   Omniverse/Isaac modules.
2. Configure the stage and fixed physics step explicitly. Load assets and
   initialize/reset the simulation before reading articulation state.
3. Step physics deliberately; rendering callbacks are not a simulation clock.
4. Shut down the application in a guaranteed cleanup path, including on errors.

For a headless adapter, select `headless: true` in its `SimulationApp` config.
Do not assume `SimulationApp.update()` alone is a correctly configured fixed
physics step. The future adapter must use the selected simulation API's
documented stepping operation and verify it with a step-count test.

## 5. Keep environments separate

| Terminal/process | Environment | Responsibility |
| --- | --- | --- |
| Native build and C++ worker | Normal compiler/CMake environment | FleetSyncSim, protocol validation, GNSS models, localization |
| Isaac application | Isaac's `python.sh --no-ros-env` launcher | Stage, physics, articulation state, adapter-side I/O |
| Optional future ROS workflow | Explicitly approved matching ROS/Isaac versions | Only after a concrete ROS requirement is promoted |

Do not globally export Isaac's `PYTHONPATH`/`LD_LIBRARY_PATH` or source its
environment into the native build shell. Do not install Isaac packages into
the repository's general Python environment. The first design uses a process
protocol, so it needs neither pybind11 nor a C++ extension built against
Isaac's Python ABI.

When an Isaac script starts the native worker, supply a deliberately sanitized
child environment: remove Isaac-added Python/library/plugin paths, preserve
ordinary system runtime lookup, and verify the native worker still starts.
Otherwise process separation alone does not prevent inherited-library clashes.
Prefer explicit absolute executable paths over modifying the user's shell
startup files.

## 6. Assets and experiment record

Start with a flat floor and one differential-drive robot. Pin the robot USD
and every referenced asset; record wheel-joint names, wheel radii, wheel
separation, base prim, stage units and axis orientation. Do not assume the
visual mesh's forward axis matches the articulation's base frame.

Cloud-hosted assets may require network access even when the application is
installed locally. For offline runs, prepare licensed local copies including
textures and referenced layers, and test with network access disabled. See
[NVIDIA asset access](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/installation/accessing_assets.html).
Do not commit vendor installers, asset packs, shader caches or run logs.

Record this information beside each run, outside deterministic reference traces:

```text
repository commit and worktree state
Isaac release and build identifier
installer/container digest; asset paths and content hashes
OS, kernel, GPU, driver, native compiler and build preset
bundled Python version and enabled extension versions
physics dt, solver/device settings, render settings and control period
map/scenario hashes, coordinate anchor and robot-to-prim mapping
resolved FleetSyncSim seed; any separate physical-simulator seed/settings
protocol version and recorded input journal hash
```

Wall-clock timestamps and host diagnostics belong to this experiment record,
not to simulation decisions or byte-stable structured traces.

## Troubleshooting

| Symptom | First check | Do not do |
| --- | --- | --- |
| Missing `isaacsim`/`omni` module | Bundled launcher and SimulationApp import order | Install unrelated packages into system Python |
| Missing native symbol or wrong shared library | Inherited library paths and exact Isaac build | Copy random `.so` files between environments |
| Robot looks 100 times too large | Stage meters-per-unit and asset transforms | Rescale map costs to compensate |
| Robot drives sideways or turns the wrong way | Base axis, joint signs, wheel order and heading conversion | Negate axes until the scene merely looks plausible |
| Missing textures or unresolved USD references | Full local asset dependency set and access permissions | Treat a partially loaded scene as a passed test |
| Timing changes when rendering is disabled | Physics stepping and integer timestamp mapping | Use render FPS as logical time |
| Native editor shows missing-header errors | Active CMake preset and generated compile database | Change domain includes to absolute paths |

Setup is complete only after the native baseline, compatibility checker,
stock scene and standalone stepping check pass. Integration itself remains
future work until the stage in the overview is authorized and implemented.