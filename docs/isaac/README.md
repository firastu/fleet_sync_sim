# Isaac Sim Integration Guide

Status: stage 1 (read-only replay) was explicitly promoted by the owner on
2026-09-21 and implemented under [ADR-020](../design_decisions/ADR-020-isaac-stage1-readonly-replay-export.md).
Stages 2 and 3 remain proposals — see the bounded exception recorded in the
[current milestone](../CURRENT_MILESTONE.md).

Audience: a developer comfortable with basic C++, Python, Git and a Linux
terminal, but new to Isaac Sim and this repository.

## Reading order

1. Read this overview for scope and the existing API gaps.
2. Follow [workstation setup](SETUP.md), including the no-GPU fallback.
3. Review the [proposed architecture](ARCHITECTURE.md) before designing code.
4. Implement the approved stage using the ticket-sized
   [implementation checklist](IMPLEMENTATION.md), with tests and stop conditions.

## Scope and starting point

This guide concerns NVIDIA **Isaac Sim**, the physical/sensor simulator. It
does not require Isaac Lab, reinforcement learning, ROS 2 or a cloud service.
Isaac Sim is a development backend, not a runtime requirement for the robot
autonomy core.

Only stage 1 is implemented scope. Stages 2 (sensor harness) and 3 (physical
motion backend) stay gated: the owner must promote each bounded stage into
the milestone and accept its durable contracts in the next available ADR.
Read the applicable [engineering rules](../../AGENTS.md) before changing code.

Stage 1 delivery:

```text
apps/fleet_isaac_export/   opt-in exporter (FLEET_BUILD_ISAAC_TOOLS, default OFF)
scenarios/isaac_replay.json  straight/turn/stop fixture run with GNSS belief
tools/isaac/               pure conversion + export validation + Isaac viewer
tests/unit/isaac/          native adapter tests
```

The Isaac rendering path of `tools/isaac/replay.py` has NOT been executed on
a supported GPU workstation yet; it must pass the smoke test in
[SETUP.md](SETUP.md) before the bridge is claimed working end to end.

## Fastest useful sequence

| Stage | Deliverable | What it proves | What it does not prove |
| --- | --- | --- | --- |
| 1. Read-only replay | Display an existing FleetSyncSim run in Isaac | Coordinate conversion, robot identity and visualization work | Physics, sensor realism or closed-loop autonomy |
| 2. Sensor harness | Move one physical robot; deliver modeled GNSS and relative odometry to a C++ localization harness | Truth/measurement separation, outage and drift with physical motion | The current graph Robot can drive a physical vehicle |
| 3. Physical motion backend | Route intent becomes physical commands and explicit completion feedback | One owner of physical motion; end-to-end execution | Estimator selection, ROS integration or real hardware readiness |

Finish and review each stage before beginning the next. Do not start by
rewriting the event queue, embedding Isaac inside the domain libraries, or
turning the scenario runner into a vehicle controller.

## Existing code versus missing integration

| Existing surface | Can be reused | Missing or unsafe to assume |
| --- | --- | --- |
| [ScenarioRunner](../../include/fleet/scenario/scenario_runner.hpp) | Bounded reference runs, stepping, trace sinks, read-only `truth_pose_for` (ADR-020) | No pose/odometry injection API; `robot()` returns a const reference |
| [Robot](../../include/fleet/robot/robot.hpp) | Mission, routes, map knowledge, GNSS sample retention | Movement is timed graph traversal, not wheel control or physical arrival feedback |
| [LocalizationTracker](../../include/fleet/localization/estimate_tracker.hpp) | Apply fixes and propagate relative odometry in a standalone C++ harness | Not a Python binding; Robot exposes its tracker read-only |
| [Pose types](../../include/fleet/localization/pose.hpp) | Explicit truth versus estimate, timestamp and heading vocabulary | No 3D pose, altitude, covariance, IMU fusion or global map projection |
| [GNSS models](../../include/fleet/localization/gnss_model.hpp) | Perfect/noisy/unavailable measurement behavior | Isaac body pose is truth, not automatically a GNSS fix |
| [Truth adapter](../../include/fleet/world/truth_pose.hpp) | Reference-backend truth for tests and replay | It derives graph truth, not the actual pose of an Isaac rigid body |

The dead-reckoning adapter in [robot odometry](../../src/robot/odometry.cpp)
currently derives motion from local graph geometry. External encoder data must
not be added on top of that propagation: that would count motion twice.

The current planner does not navigate from the localization estimate. A
successful sensor harness therefore demonstrates localization behavior, not
localization-dependent route execution.

## Tooling decision: no Autoproj for now

Keep the current CMake presets for C++. Use Isaac's own supported Python
environment for Isaac-side scripts. Exchange data across an explicit adapter
boundary; do not merge their library search paths or dependency installations.

[Autoproj](https://www.rock-robotics.org/documentation/autoproj/) manages sets of
source packages and their build/OS dependencies. It does not provide an Isaac
sensor bridge, a coordinate system, a physical controller, or a solution to
Python ABI compatibility. This repository currently has one CMake project and
no Rock/Orocos workspace that needs orchestration.

Adding Ruby, package sets and a second workspace manager now would create more
setup work than it removes. No Autoproj installation or `docs/autoproj/`
workspace recipe is needed for the proposed first integration.

Reconsider it only when several independently versioned native repositories
must be built together, particularly if Rock/Orocos dependencies are adopted.
At that point, first inventory those packages and their dependency conflicts;
then compare Autoproj with the team's existing workspace tooling. An Autoproj
recipe must preserve plain CMake builds and keep NVIDIA downloads, licensing,
drivers and Isaac's Python environment outside the source-package build graph.

## Completion criteria

A developer should be able to build and test FleetSyncSim on a machine without
Isaac installed. GPU-dependent checks belong to an explicit optional workflow.
Reference traces must retain their existing determinism guarantee; physical
simulation uses separately declared tolerances and a recorded environment.

Do not claim Isaac integration is working until the workstation smoke test and
the selected stage's acceptance tests have actually run on a supported GPU.
The documentation alone is not that evidence.