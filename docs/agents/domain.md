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
├── CONTEXT.md                              ← glossary
└── docs/
    ├── README.md                           ← start here (index + reading order)
    ├── ADR-001 … ADR-009-*.md
    ├── prd.md
    ├── architecture.md
    ├── setup-windows-wsl.md
    ├── test-plan.md
    ├── milestones.md
    ├── data-model.md
    ├── schema.sql
    ├── _archive/                           ← superseded docs (ROS 2 / Nav2 plan); do not use
    └── agents/
        ├── issue-tracker.md
        └── domain.md
```

New ADRs continue the existing numbering (`ADR-010-…`) and live directly in `docs/`, not in a `docs/adr/` subfolder.

## Use the glossary's vocabulary

When your output names a domain concept (in an issue title, a refactor proposal, a hypothesis, a test name), use the term as defined in `CONTEXT.md`. Don't drift to synonyms the glossary explicitly avoids.

If the concept you need isn't in the glossary yet, that's a signal: either you're inventing language the project doesn't use (reconsider) or there's a real gap (note it for `/domain-modeling`).

## Flag ADR conflicts

If your output contradicts an existing ADR, surface it explicitly rather than silently overriding:

> _Contradicts ADR-004 (local planner), but worth reopening because…_
