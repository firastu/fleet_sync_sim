# FleetSyncSim Documentation

FleetSyncSim documentation is intentionally split by purpose and authority.

The goal is to keep architectural contracts precise while preventing contributors and coding agents from loading the entire project history for ordinary tasks.

---

## Document authority

| Location                                           | Purpose                                     | Authority                                        |
| -------------------------------------------------- | ------------------------------------------- | ------------------------------------------------ |
| `../AGENTS.md`                                     | Contributor and coding-agent rules          | **Normative**                                    |
| `CURRENT_MILESTONE.md`                             | Current implementation scope                | **Normative**                                    |
| `design_decisions/ADR-*.md`                        | Accepted durable architecture decisions     | **Normative**                                    |
| Architecture documents explicitly marked normative | Current system contracts                    | **Normative where stated**                       |
| `AI_CONTEXT.md`                                    | Compact architecture/status orientation     | Non-normative summary                            |
| `PROJECT_VISION.md`                                | Long-term direction / north star            | Directional                                      |
| Roadmaps                                           | Possible staged evolution                   | Directional                                      |
| Research documents                                 | Evidence, literature, exploratory reasoning | Non-normative                                    |
| Tooling documents                                  | Development and external-tool guidance      | Non-normative unless explicitly stated otherwise |

Code and tests are the executable realization of these contracts.

If implementation and normative documentation disagree, treat that as a defect. Determine which contract is intended and make them consistent rather than silently choosing one.

---

## Implementation flow

Research and future ideas do not directly become implementation scope.

```text
research / idea
      |
      v
engineering question
      |
      v
experiment / design
      |
      v
ADR if a durable contract is required
      |
      v
CURRENT_MILESTONE
      |
      v
implementation + tests
```

A capability appearing in `PROJECT_VISION.md`, a research document, a tooling document, or an external framework plan does **not** authorize its implementation.

---

## Default context for coding agents

Do not read all repository documentation before every task.

Normal startup context is:

```text
1. ../AGENTS.md
2. CURRENT_MILESTONE.md
3. AI_CONTEXT.md
4. only ADRs relevant to the task
5. source files and tests being changed
```

If `.ai/SESSION.md` exists, it may be read after checking Git state.

It is local, non-authoritative handoff context.

Before trusting any conversation summary or session handoff:

```sh
git status
git log -5 --oneline
```

Git and normative repository documentation take precedence over conversation history or local notes.

---

## Reading ADRs efficiently

Accepted architecture decisions live under:

```text
design_decisions/ADR-NNN-*.md
```

Do not preload every ADR.

Discover available ADRs directly from the repository:

```sh
find docs/design_decisions -maxdepth 1 -name 'ADR-*.md' -printf '%f\n' | sort
```

Search by topic when necessary:

```sh
rg -n "localization|GNSS|movement|network|map|scenario|robot" \
    docs/design_decisions/ADR-*.md
```

Then open only the relevant decisions.

See:

```text
design_decisions/README.md
```

for ADR usage rules.

---

## Document responsibilities

Each document type has one primary job.

### `AGENTS.md`

Defines **how work is performed**:

* scope discipline;
* Git discipline;
* determinism requirements;
* validation requirements;
* architectural boundaries;
* agent reporting rules.

### `CURRENT_MILESTONE.md`

Defines **what may be implemented now**.

It should contain:

* the active objective;
* current milestone invariants;
* completed steps directly relevant to the active milestone;
* allowed work;
* explicitly deferred work;
* the validation gate.

It is not a historical changelog.

### `AI_CONTEXT.md`

Provides compact orientation.

It summarizes enough architecture for an agent to locate the correct module and ADR without reconstructing the entire project from historical documents.

It is a cache, not an authority.

### `design_decisions/`

Contains durable architectural contracts.

If future implementation will rely on a decision, the ADR is the correct place for its precise rationale, constraints, and consequences.

### `PROJECT_VISION.md`

Explains why the project exists and where it may eventually go.

It should change slowly.

It must not become a detailed record of current implementation state.

### Root `README.md`

Introduces the project to humans.

It should explain:

* what FleetSyncSim is;
* major implemented capabilities;
* how to build it;
* how to run it;
* where deeper documentation lives.

It is not a second architecture database.

### Research documents

Research contains:

* literature;
* evidence;
* hypotheses;
* exploratory engineering reasoning.

Research does not authorize implementation.

### Tooling documents

Tooling documents describe workflows and integration boundaries around the core.

Examples include GIS workstations, map inspection workflows, future external simulator setup, and similar development infrastructure.

Tooling documents do not automatically create runtime dependencies or implementation scope.

External simulator workflow:

* [Isaac Sim integration guide](isaac/README.md): workstation setup, adapter
    decisions, junior-oriented implementation packages and acceptance
    tests. Stage 1 (read-only replay) is implemented scope under
    [ADR-020](design_decisions/ADR-020-isaac-stage1-readonly-replay-export.md);
    later stages remain guidance for future promotions, not accepted runtime
    architecture. The guide also records why Autoproj is not needed for the
    initial integration.

---

## Duplication rule

Avoid maintaining the same detailed fact in several files.

Prefer:

```text
current implementation scope
    -> CURRENT_MILESTONE.md

durable architectural decision
    -> design_decisions/ADR-*.md

compact agent orientation
    -> AI_CONTEXT.md

long-term direction
    -> PROJECT_VISION.md

human build/run instructions
    -> ../README.md

external-tool workflow
    -> tooling document

research evidence
    -> research/
```

Link to the authoritative source instead of copying its full reasoning.

---

## ADR discipline

ADRs are first-class architecture documentation.

They are:

* sequentially numbered;
* normative once accepted;
* focused on durable decisions;
* not required merely because a commit is large.

When exact semantics matter, read the relevant ADR rather than relying on a summary in another document.

If a durable decision changes substantially, prefer an explicit superseding or follow-up ADR rather than rewriting history to make it appear the later architecture always existed.

---

## Staleness rule

A dangling reference to a nonexistent file is a documentation bug.

A document describing already-completed work as future work is also a documentation bug.

When a milestone materially changes project architecture:

1. update the relevant ADR or create a new one if required;
2. update `CURRENT_MILESTONE.md`;
3. refresh `AI_CONTEXT.md` only where the compact architecture snapshot changed;
4. update the root README only if human-facing capabilities or usage changed;
5. update specialized tooling documents only when their actual boundary or workflow changed.

Do not rewrite unrelated historical documents merely to mention the latest commit.
