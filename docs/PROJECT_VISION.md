# FleetSyncSim Project Vision

## North star

FleetSyncSim exists to explore software architectures for:

> Sovereign PNT, mapping, navigation, and cooperative autonomy for robots.

"Sovereign" does not mean rejecting useful external systems.

It means that essential robot capabilities should not contain unnecessary
hard runtime dependencies on infrastructure outside the autonomous system.

A robot should degrade gracefully when GNSS, cloud services, map servers,
control stations, or peer connectivity become unavailable.

---

# Capability pillars

## 1. Sovereign maps and navigation

Robots should be able to operate from locally available map data and locally
executed planning algorithms.

Target architecture:

    geographic/map data
            ↓
      local map model
            ↓
      routing/planning
            ↓
      robot navigation

Core operation must not require an online map or routing provider.

---

## 2. Resilient positioning and navigation

GNSS is a useful positioning source, not an assumed permanent dependency.

Long-term positioning may combine:

    GNSS
    IMU
    wheel odometry
    visual odometry
    LiDAR odometry
    map matching
    environmental features
            ↓
      state estimation
            ↓
       pose + velocity
       + uncertainty

When GNSS disappears, navigation quality may degrade, but position should not
immediately become undefined.

---

## 3. Local world models

Each robot maintains its own knowledge.

The simulator distinguishes:

    simulation ground truth

from:

    robot-local belief

Robots should eventually reason about:

- static map information;
- dynamic obstacles;
- observations;
- provenance;
- information age;
- pose estimates;
- uncertainty.

---

## 4. Direct robot-to-robot cooperation

Robots should eventually be capable of exchanging useful information directly.

Potential information includes:

- map deltas;
- obstacle observations;
- pose estimates and uncertainty;
- landmarks;
- mission/route intentions;
- health/status information.

The architecture must not require:

    Robot -> Cloud -> Robot

for local cooperation.

---

## 5. Graceful degradation

Loss of one capability should not necessarily collapse the whole system.

Examples:

    GNSS lost
        ↓
    dead reckoning / local sensing / map matching
        ↓
    uncertainty increases
        ↓
    autonomy adapts

and:

    V2V connection lost
        ↓
    robots continue independently
        ↓
    observations remain local
        ↓
    connectivity returns
        ↓
    knowledge reconciles

---

# Long-term system model

                     Mission / Autonomy
                            |
                     Route + Planning
                            |
                +-----------+-----------+
                |                       |
           Map System             Localization / PNT
                |                       |
         offline maps               GNSS
         dynamic state              IMU
         map matching               odometry
         map merging                perception
                |                       |
                +-----------+-----------+
                            |
                    Local World Model
                            |
                      V2V Cooperation
                     /       |       \
               map delta    pose    observations
                            |
                   Communication Layer
                            |
                         Robots

---

# Important scope rule

This document describes direction, not current implementation scope.

A capability becomes implementable only when promoted into
`CURRENT_MILESTONE.md` or explicitly requested.

See the roadmaps and research documents for possible future approaches.