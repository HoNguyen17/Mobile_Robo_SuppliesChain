# ADR-006: Multi-Robot Rollout — Staged and Gated (A → B → C)

## Context
Multiple independently-navigating robots increase both Nav2 configuration complexity (namespacing, TF prefixes) and Unity-bridge load (one TCP connection per robot, per `docs/architecture.md` §7), with an 8-week timeline and a 4-person team. Attempting 3 robots outright risks jeopardising the graded M1–M3 milestones, which only require one robot plus scripted NPC obstacles.

## Decision
Multi-robot capability is staged and gated:
- **Stage A:** 1 robot + scripted NPC obstacles (Unity-only, not ROS nodes). This is the M3 baseline and is sufficient to meet all graded milestones.
- **Stage B:** add one independently-navigating second robot (`/robot2`, TF prefix `robot2/`), entered only if the Stage A exit gate is met (`docs/milestones.md`).
- **Stage C:** add a third robot (`/robot3`, TF prefix `robot3/`), entered only if the Stage B exit gate is met.

If a stage's entry gate fails, the project ships the prior stage as final — Stage B or C are never required to meet the graded milestones.

## Consequences
- Grading risk is minimised: the team can always fall back to a fully working Stage A submission regardless of how far Stage B/C progress gets.
- Namespacing and TF-prefix conventions must be decided once (`docs/architecture.md` §7) and applied consistently before Stage B is attempted, rather than improvised per robot.
- The ROS-TCP-Endpoint bandwidth/connection-count question (`docs/architecture.md` §7, [VERIFY]) must be resolved before Stage B begins, since it determines whether one endpoint process is reused or one-per-robot is required.

## Alternatives Rejected
- **Build for 3 robots from the start:** rejected — front-loads integration risk onto a graded path that only requires 1 robot, with no partial-credit fallback if it fails.
- **Skip multi-robot entirely:** rejected — the course brief requires documenting the staged approach even though it is not required for the graded milestones; Stage A alone already satisfies M3, so Stage B/C are pursued only as time permits.
