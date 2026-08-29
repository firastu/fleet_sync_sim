# FleetSyncSim Project Vision

## North star

FleetSyncSim explores software architectures for:

> **Sovereign positioning, mapping, navigation, and cooperative autonomy
> for robots.**

"Sovereign" does not mean avoiding external systems.

It means that essential autonomous capabilities should not contain unnecessary
hard runtime dependencies on infrastructure outside the robot or fleet.

External positioning, maps, control stations, and network infrastructure may
improve capability, but their temporary loss should produce controlled
degradation rather than immediate system collapse.

---

## Capability pillars

### 1. Locally controlled maps and navigation

Robots should be able to operate from locally available map data and locally
executed planning algorithms.

Conceptually:

    source geographic data
            ↓
       map pipeline
            ↓
      local map model
            ↓
      routing/planning
            ↓
          robot

Runtime navigation should not inherently depend on a cloud map or routing
provider.

---

### 2. Resilient positioning and navigation

GNSS is one potential positioning source, not an assumed permanent dependency.

Long-term experiments may combine:

    GNSS
    IMU
    wheel odometry
    visual odometry
    LiDAR odometry
    environmental/map matching
             ↓
       state estimation
             ↓
    pose + velocity + uncertainty

When an absolute positioning source disappears, the system should model
degrading certainty rather than instantaneous loss of all positional knowledge.

---

### 3. Robot-local world models

Every robot maintains its own knowledge.

The architecture distinguishes:

    simulation ground truth

from:

    robot-local belief

Robot-local state may eventually contain:

- static map information;
- dynamic map changes;
- observations;
- provenance;
- information age;
- pose estimates;
- uncertainty.

Temporary disagreement between robots is expected in a distributed system.

---

### 4. Peer-to-peer cooperation

Robots should eventually be able to exchange useful state directly.

Possible information includes:

- map deltas;
- obstacle observations;
- pose estimates and uncertainty;
- landmarks;
- route or mission intent;
- health and status.

Local cooperation must not fundamentally require:

    Robot -> Cloud -> Robot

A command station may assist or aggregate information, but should not be a
mandatory intermediary for robot-to-robot cooperation.

---

### 5. Graceful degradation

The system should explicitly study degraded modes.

Example:

    GNSS unavailable
        ↓
    dead reckoning / local sensing / map matching
        ↓
    uncertainty grows
        ↓
    autonomy adapts

Likewise:

    V2V unavailable
        ↓
    robots continue independently
        ↓
    knowledge diverges
        ↓
    connectivity returns
        ↓
    knowledge reconciles

---

## Long-term conceptual stack

                 Mission / Autonomy
                        |
                 Route + Planning
                        |
            +-----------+-----------+
            |                       |
        Map System             Localization / PNT
            |                       |
      offline maps                  GNSS
      dynamic state                 IMU
      map matching                  odometry
      map merging                   perception
            |                       |
            +-----------+-----------+
                        |
                Local World Model
                        |
                  V2V Cooperation
                 /       |       \
          map deltas    pose    observations
                        |
                Communication Layer
                        |
                      Robots

---

## What FleetSyncSim is today

The current simulator does not attempt to implement this complete stack.

Its first foundational problem is:

> **Distributed robot-local map knowledge under unreliable communication.**

This establishes several prerequisites for later sovereign-autonomy work:

- local knowledge instead of global shared state;
- deterministic simulation;
- unreliable peer communication;
- partitions;
- reconciliation;
- autonomous replanning;
- reproducible scenario execution.

Future capabilities must be introduced incrementally and only when promoted
into the active milestone.

---

## Guiding principle

> A capability should continue locally where reasonably possible when an
> external service disappears, and the system should make degradation
> observable rather than hiding it.