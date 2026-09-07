# ADR-003: ROS 2 Distribution — Pinned in Phase 0, Not Assumed

## Context
The M1 milestone adapts the Unity Robotics Hub's Nav2 + SLAM example. That tutorial repository's current branch determines which ROS 2 distributions it actually supports, and this can change independently of this document set. Asserting a distro now, without checking the repository, risks the whole plan being built on an incompatible assumption.

## Decision
The ROS 2 distribution is **not** asserted as fact in this document set. It is a Phase 0 output: the team clones the tutorial repository, reads its supported-distro documentation/CI configuration, and pins one specific distro (and, where relevant, a specific commit or tag of the tutorial) before any M1 work begins. [VERIFY] — Phase 0 task: "Confirm ROS 2 distro and tutorial commit compatibility; record the result here and in `docs/milestones.md` Phase 0 exit gate."

Candidate (for planning purposes only, not yet confirmed): a current ROS 2 LTS distribution compatible with Unity-Robotics-Hub's `ROS-TCP-Connector`/`ROS-TCP-Endpoint` and with Nav2. This candidate must be verified, not assumed, in Phase 0.

## Consequences
- All distro-specific names in `docs/architecture.md` (e.g. `behavior_server` vs. an older `recoveries_server` naming, exact default parameter file names) carry a [VERIFY] tag until Phase 0 confirms them.
- The Phase 0 exit gate in `docs/milestones.md` cannot be met without this decision being finalised and documented as an update to this ADR (status change, not a new ADR).
- If the tutorial repository only supports a distro the team cannot install in the available environment, Phase 0 must surface that risk immediately, before any M1 implementation time is spent.

## Alternatives Rejected
- **Assume the most recent ROS 2 LTS release without checking the tutorial repo:** rejected — directly contradicts the course brief's instruction not to assert a distro as fact, and risks a Phase 1 restart if the tutorial does not support it.
- **Defer distro selection past Phase 0:** rejected — every other phase depends on Nav2/tutorial APIs that are distro-specific; deferring further would stall M1 adaptation.
