# AGENTS.md — Engineering Rules for Contributors and Coding Agents

Normative.

This file defines how work is performed in this repository by humans and AI
agents. Document authority is defined in `docs/README.md`.

If two normative documents disagree, stop and fix the disagreement before
building new behavior.

---

## 0. Context-loading protocol

Do not load the entire repository documentation before every task.

At the start of a coding session:

1. inspect the repository, because the owner may have committed or pushed
   since the previous session:

   ```sh
   git status
   git log -5 --oneline
   ```

2. read, in this order:

   ```text
   AGENTS.md
   docs/CURRENT_MILESTONE.md
   docs/AI_CONTEXT.md
   ```

3. read only the ADRs relevant to the subsystem being changed;

4. inspect the implementation and tests directly involved in the task.

Read `PROJECT_VISION.md`, roadmaps, research documents, or tooling documents
only when the task requires long-term design context.

Do not preload every ADR or research document "just in case".

`docs/AI_CONTEXT.md` is an orientation aid, not an authority source. If it
conflicts with `AGENTS.md`, `CURRENT_MILESTONE.md`, an ADR, or the code/tests,
the authoritative source wins and the stale context document should be
corrected.

Never assume conversation history is newer than the repository.

If `.ai/SESSION.md` exists, it may be read after repository inspection as a
local session handoff. It is non-authoritative and must never override Git or
normative documentation.

---

## 1. Scope discipline

`docs/CURRENT_MILESTONE.md` defines normal implementation scope.

Do not implement a capability solely because it appears in:

* `PROJECT_VISION.md`;
* a roadmap;
* research notes;
* tooling documentation;
* a future milestone;
* an external framework or simulator.

Follow the current milestone in order. Do not silently skip ahead in a
milestone ladder.

New durable architectural contracts require an ADR:

```text
docs/design_decisions/ADR-NNN-*.md
```

ADR numbers are sequential and never reserved.

Do not create an ADR merely because a change is large. Create one when future
work will rely on the decision as a contract.

Keep feature changes review-sized. A feature commit should normally be
reviewable in one sitting.

---

## 2. Git discipline

The AI implements; the human reviews before feature commits unless explicitly
instructed otherwise.

Before changing Git state, inspect it.

Never without explicit instruction:

* push;
* rebase;
* amend;
* reset published history;
* force-push;
* change remotes;
* change Git configuration.

Stage files explicitly by path.

Never use:

```sh
git add -A
```

Generated outputs, traces, and build directories are not committed unless they
are intentional test fixtures.

Commit messages use:

```text
feat(scope): summary
fix(scope): summary
docs(scope): summary
```

The body explains the engineering reason and important contracts.

Do not add AI attribution trailers, tool metadata, or generated-by text.

---

## 3. Determinism discipline

Deterministic reference behavior is a core project property.

Observable simulation behavior must be a function of:

```text
(scenario, resolved seed)
```

unless an accepted ADR explicitly introduces another input.

Do not allow the following to influence behavior or deterministic output:

* wall-clock time;
* pointer values;
* unordered-container iteration order;
* unspecified random-distribution implementations;
* hidden global state.

Simulation randomness flows through:

```text
fleet::simulation::DeterministicRng
```

Do not use standard-library probability distributions in deterministic
simulation paths.

Random draw consumption is part of a model's contract when changing it could
shift later deterministic outcomes.

Same scenario + same resolved seed must produce byte-identical structured
traces where the relevant ADR promises this.

If output intentionally changes, state why and prove non-regression for
behavior intended to remain unchanged.

Self-scheduling effects must not create zero-time retry loops. Repeating effects
schedule strictly later than their predecessor unless same-tick behavior is an
explicit event-ordering contract.

---

## 4. Architectural boundaries that must not blur

### Robot autonomy — ADR-007

Robots do not know about:

* networking implementation;
* the event queue;
* scenario files;
* visualization;
* external simulators.

Robots are driven through their public API.

### Scenario runner — ADR-009/010

`ScenarioRunner` owns orchestration and wiring.

It must not absorb domain logic belonging to:

* Robot;
* planning;
* reconciliation;
* networking;
* ControlStation;
* localization models.

Injected events and loaded scenario events use the same effect path.

### Observation and export

Trace sinks, GeoJSON exporters, consoles, and debugging tools observe state.

Observation must not change simulation behavior.

### World truth — ADR-011

Simulation ground truth is not directly available to robot autonomy.

Truth reaches robot belief only through an explicit sensing or measurement
boundary.

Robots do not depend on `fleet::world`.

### Localization — ADR-016/017

`GroundTruthPose` represents simulator truth.

`LocalizationEstimate` represents information available through the
localization boundary.

A localization estimate's `estimated_at` is the time the estimate refers to,
not the time a caller happens to inspect it.

Future localization consumers must not bypass the measurement boundary to read
truth directly.

### External simulation and hardware

External systems such as NVIDIA Isaac Sim, ROS 2, Gazebo, or future physical
robots are adapter-side systems.

They must not redefine FleetSyncSim domain types or become hidden dependencies
of the deterministic core.

They become implementation scope only when promoted into
`CURRENT_MILESTONE.md`.

---

## 5. Build and validation discipline

Before a feature commit, all required presets must pass:

* `debug` with warnings as errors;
* `asan` with ASan+UBSan;
* `tsan`.

For the current GCC 13 environment, TSan requires the existing per-process
workaround:

```sh
setarch $(uname -m) -R ...
```

Do not change host `sysctl` settings.

Check command exit codes explicitly.

Do not trust a binary that may be stale.

When fixture copying, generated configuration, or sanitizer state is suspect,
reconfigure or rebuild before diagnosing the product code.

Every feature must add tests at the level where its behavior lives:

* domain semantics -> unit tests;
* orchestration -> runner/integration tests;
* declarative behavior -> scenario tests;
* serialization contracts -> output tests.

Preserve the founding trace when behavior outside its scenario is intended to
remain unchanged.

Run:

```sh
git diff --check
```

before review.

---

## 6. Dependencies and tooling

Environment:

* C++20;
* GCC 13;
* CMake >= 3.21;
* preset-based builds.

New third-party dependencies must be:

* justified;
* pinned;
* minimal;
* scoped to the target that consumes them.

Parser/tool dependencies remain `PRIVATE` where possible and must not leak into
public domain headers.

Do not introduce formatting, linting, build-system, GIS, ROS, or simulator
dependencies merely for convenience.

Code follows existing repository conventions:

* `fleet::<module>` namespaces;
* strong types from `fleet/common`;
* contract-first public documentation;
* explicit deterministic behavior;
* existing ADR decisions.

---

## 7. Documentation discipline

Documents have narrow jobs:

```text
AGENTS.md
    how work is performed

CURRENT_MILESTONE.md
    what may be implemented now

AI_CONTEXT.md
    compact orientation for contributors/agents

design_decisions/
    durable accepted contracts

PROJECT_VISION.md
    long-term direction

research/
    evidence and exploration

tooling documents
    external development workflows
```

Do not duplicate detailed implementation history across several documents.

`CURRENT_MILESTONE.md` records the active milestone, not a complete project
changelog.

`README.md` is for humans discovering and running the project, not a second
architecture database.

A reference to a nonexistent file is a documentation bug.

When an architectural milestone materially changes the project topology,
update `docs/AI_CONTEXT.md` in the same documentation scope.

---

## 8. Agent reporting discipline

Do not repeat the entire project history after every task.

A review report should normally contain only:

1. behavior/contracts changed;
2. files changed;
3. tests added or changed;
4. validation results;
5. determinism/compatibility impact;
6. ADR/documentation impact;
7. blockers;
8. proposed commit scope.

Use exact facts rather than long project recaps.
