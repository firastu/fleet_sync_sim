# AGENTS.md — Engineering Rules for Contributors and Coding Agents

Normative. This file constrains how work is done in this repository,
for humans and AI agents alike. Authority levels of all documents are
defined in [docs/README.md](docs/README.md); when this file and another
normative document disagree, fix the disagreement in a docs commit
before writing code.

## 1. Scope discipline

- [docs/CURRENT_MILESTONE.md](docs/CURRENT_MILESTONE.md) defines the
  normal implementation scope. Do not implement a feature solely
  because it appears in a vision, roadmap, research, or tooling
  document.
- New architectural contracts require an ADR
  (`docs/design_decisions/ADR-NNN-*.md`, sequential numbering, never
  reserved). Do not write an ADR merely because a commit feels big —
  only for durable contracts others will build against.
- Keep changes milestone-relevant and review-sized. One feature commit
  should be reviewable in one sitting.

## 2. Git discipline

- The AI implements; the human reviews before every feature commit.
  Never commit work that has not been reviewed unless explicitly told
  to.
- Never push, rebase, amend, or otherwise rewrite history without
  explicit instruction. Check `git status` and `git log` first: the
  owner sometimes commits and pushes between sessions.
- Commits carry no AI-attribution trailers or tool metadata. Message
  style: `feat(scope): summary`, imperative, body explains why.
- Stage files explicitly by path. Never `git add -A`. Generated outputs
  (traces, build dirs) are never committed; ignore rules stay narrowly
  scoped.

## 3. Determinism discipline (the project's core property)

Every observable behavior is a pure function of `(scenario, resolved
seed)`:

- No wall-clock time, no pointer values, no unordered-container
  iteration may influence behavior or any output (including the trace).
- No `std` distributions in simulation code; randomness flows only
  through `fleet::simulation::DeterministicRng` (ADR-006).
- Same scenario + same seed must produce byte-identical traces. When a
  change intentionally alters output, say so in the commit message and
  prove non-regression where behavior is meant to be unchanged.
- Self-scheduling effects must always land strictly later than their
  predecessor; validate timing settings at a single choke point
  (see ADR-010 for the movement pattern).

## 4. Boundaries that must never blur

- **Robot autonomy (ADR-007):** robots know nothing about networking,
  the event queue, the scenario, or visualization. They are driven
  through their public API.
- **Wiring lives in the scenario runner (ADR-009/010):**
  `ScenarioRunner` schedules and wires; it must not absorb robot,
  network, station, reconciliation, or planning logic.
- **Observation is read-only:** trace sinks and future exporters
  observe; they never influence a run.
- **World truth (boundary planned with milestone M2):** simulation
  ground truth is never directly accessible to robots — only through an
  observation model. The durable contract will be recorded when the
  world/sensing commit lands; do not couple robots to world state in
  the meantime.

## 5. Build and validation discipline

- Three presets must pass before any feature commit: `debug`
  (`-Werror`), `asan` (ASan+UBSan), `tsan`. TSan on GCC 13 requires
  `setarch $(uname -m) -R` per-process (never change sysctl).
- Check build and test exit codes explicitly; never trust output of a
  possibly-stale binary. When in doubt, rebuild cleanly.
- Every feature adds or extends tests at the same level the behavior
  lives (unit for domain semantics, runner-level for orchestration,
  on-disk scenario files for the declarative contract).

## 6. Dependencies and tooling

- New third-party code must be pinned (FetchContent tag), minimal, and
  justified in the introducing commit; parser-type dependencies are
  `PRIVATE` to the consuming target and never leak into public headers.
- Environment: C++20, GCC 13, CMake >= 3.21 presets. Do not introduce
  clang-format/clang-tidy/ninja config without instruction.
- Code style follows the existing tree: `fleet::<module>` namespaces,
  contract-first doc comments on public types, strong types from
  `fleet/common`, no exceptions to ADR decisions.

## 7. Docs hygiene

- Update `CURRENT_MILESTONE.md` only to reflect genuinely satisfied (or
  re-scoped) criteria, not aspirations.
- `docs/geospatial.md` describes the tooling boundary between external
  GIS tools and this core; simulator code never depends on GIS tooling.
- A reference to a file that does not exist is a bug: fix dangling doc
  references in the same commit that introduces them.
