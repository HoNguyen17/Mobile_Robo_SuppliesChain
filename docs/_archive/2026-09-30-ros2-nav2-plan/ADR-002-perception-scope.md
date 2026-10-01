# ADR-002: Perception Scope — Known Shelf-Slot Coordinates, No ML Detection in the Graded Path

## Context
Item classification and perception-driven detection are explicitly ungraded per the course brief; grading centres on obstacle-avoidance planning. Building an ML perception pipeline would consume time better spent on M1–M3.

## Decision
Shelf-slot coordinates come from the item catalog (`shelf_slots`/`items` tables, `docs/data-model.md`), not from any runtime perception system. No ML-based detection sits on the graded navigation path. YOLOv8n may be prototyped only as an explicitly out-of-scope stretch goal, after the M3/Stage exit gates are met (PRD FR-21).

## Consequences
- The robot always knows where to go without a perception step, so navigation testing is not confounded by detection accuracy.
- The category used for motion profiling (`docs/architecture.md` §6) is a catalog attribute, not an inferred one, which keeps that mechanism deterministic and reproducible across the N≥20 automated runs.
- Any future YOLOv8n prototype must be demonstrably isolated from the graded control loop (e.g. a separate branch or a clearly labelled optional node) so it cannot be mistaken for part of the graded path during review.

## Alternatives Rejected
- **YOLOv8n or similar CNN-based shelf/item detection on the graded path:** rejected — ungraded, adds dependency and failure-mode surface area (model weights, inference latency, dataset), and risks diverting effort from obstacle-avoidance tuning, which is what is actually assessed.
- **AprilTag/fiducial-based pose estimation:** rejected for the same reason — even a lightweight perception step is unnecessary complexity when catalog coordinates are sufficient and course-permitted.
