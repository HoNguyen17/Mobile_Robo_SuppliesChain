# Domain Docs

How the engineering skills should consume this repo's domain documentation when exploring the codebase.

## Before exploring, read these

- **`CONTEXT.md`** at the repo root (the glossary and domain overview).
- **ADRs in `docs/`**: files named `ADR-NNN-<slug>.md`. Read the ones that touch the area you're about to work in.

This repo is **single-context**: there is no `CONTEXT-MAP.md` and no per-context `docs/adr/` directories.

If any of these files don't exist, **proceed silently**. Don't flag their absence; don't suggest creating them upfront. The `/domain-modeling` skill (reached via `/grill-with-docs` and `/improve-codebase-architecture`) creates them lazily when terms or decisions actually get resolved.

## File structure

```
/
├── CLAUDE.md
├── CONTEXT.md                              ← created lazily by /domain-modeling
└── docs/
    ├── ADR-001-robot-platform.md
    ├── ADR-002-perception-scope.md
    ├── ADR-003-ros-distro-pinning.md
    ├── ADR-004-nav2-local-planner.md
    ├── ADR-005-mysql-database.md
    ├── ADR-006-multirobot-staging.md
    ├── architecture.md
    ├── data-model.md
    ├── diagrams.md
    ├── milestones.md
    ├── prd.md
    ├── schema.sql
    ├── test-plan.md
    └── agents/
        ├── issue-tracker.md
        └── domain.md
```

New ADRs continue the existing numbering (`ADR-007-…`) and live directly in `docs/`, not in a `docs/adr/` subfolder.

## Use the glossary's vocabulary

When your output names a domain concept (in an issue title, a refactor proposal, a hypothesis, a test name), use the term as defined in `CONTEXT.md`. Don't drift to synonyms the glossary explicitly avoids.

If the concept you need isn't in the glossary yet, that's a signal: either you're inventing language the project doesn't use (reconsider) or there's a real gap (note it for `/domain-modeling`).

## Flag ADR conflicts

If your output contradicts an existing ADR, surface it explicitly rather than silently overriding:

> _Contradicts ADR-004 (nav2 local planner), but worth reopening because…_
