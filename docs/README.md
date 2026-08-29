# FleetSyncSim Documentation

FleetSyncSim documentation has different authority levels.

| Location | Purpose | Authority |
|---|---|---|
| `../AGENTS.md` | Contributor / coding-agent rules | normative |
| `CURRENT_MILESTONE.md` | Current implementation scope | normative |
| `design_decisions/` | Accepted architectural contracts | normative |
| architecture documentation | Description of current system | normative where stated |
| `PROJECT_VISION.md` | Long-term direction | directional |
| roadmaps | Possible evolution | directional |
| research documents | Literature and exploratory notes | non-normative |
| tooling documents | Development/tool integration guidance | non-normative |

## Implementation flow

Research and future ideas do not directly become code.

    Research / idea
          ↓
    engineering question
          ↓
    experiment / design
          ↓
    ADR if an architectural contract is required
          ↓
    CURRENT_MILESTONE
          ↓
    implementation

## For coding agents

Read `../AGENTS.md` first.

`CURRENT_MILESTONE.md` defines normal implementation scope.

Do not implement a feature solely because it appears in a vision, roadmap,
research, or tooling document.