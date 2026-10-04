# ADR-006: Single Robot Only

**Status:** Accepted · supersedes the earlier staged multi-robot plan (Stage A/B/C)

## Context
All graded milestones (M1–M3) need only **one** robot plus moving NPCs. Multi-robot work (namespaces, TF prefixes, one bridge connection per robot) would add risk across every layer.

## Decision
Build for **exactly one robot**. No namespaces and no TF prefixes: topics are plain `/cmd_vel`, `/odom`, `/scan`. Moving obstacles are Unity-scripted **NPCs**, not robots.

## Consequences
- Simpler launch files, TF tree, bridge and metrics.
- The "Stage" concept and the old multi-robot scenario are removed from all docs.
- If multi-robot is ever needed, it will be a new ADR written after P4. Nothing is built for it in advance.
