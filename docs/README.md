# FleetSyncSim Documentation

FleetSyncSim documentation has different authority levels.

| Document | Role | Authority |
|---|---|---|
| `../AGENTS.md` | AI/contributor engineering rules | normative |
| `CURRENT_MILESTONE.md` | currently permitted implementation scope | normative |
| `design_decisions/` | accepted architectural decisions | normative |
| `architecture/` | current system contracts | normative |
| `PROJECT_VISION.md` | long-term direction | directional |
| `roadmaps/` | planned evolution | directional |
| `research/` | literature notes and experiments | non-normative |

## Important

Research material must never be interpreted as an implementation request.

The normal path is:

    Research idea
         ↓
    engineering question
         ↓
    experiment / discussion
         ↓
    ADR if architecture changes
         ↓
    CURRENT_MILESTONE
         ↓
    implementation