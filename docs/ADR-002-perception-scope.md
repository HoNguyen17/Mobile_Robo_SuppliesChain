# ADR-002: No ML Perception; Shelf Poses Come from the Catalog

**Status:** Accepted · amended 2026-10-04 ([ADR-012](ADR-012-custom-python-navigation.md), [ADR-013](ADR-013-kinematic-robot-body.md))

## Context
Item detection and classification are not graded. An ML pipeline (camera, dataset, model) would take time away from avoidance tuning.

## Decision
The robot knows where every item is from the `shelf_slots` and `items` tables. The item's category is a catalog field, not something detected. Its pose and the static map are ground truth from Unity. The only simulated sensor is the **LIDAR** (raycasts), and it is used only for navigation, to see obstacles that are not in the map.

## Consequences
- Navigation results are never mixed up with detection errors.
- Runs are deterministic and repeatable, which matters for N = 20 evidence.
- Shelf-slot poses in the database must match the Unity scene; they are exported from the scene once ([data-model.md §4](data-model.md#4-seeding)).

## Alternatives rejected
| Option | Why not |
|--------|---------|
| YOLO / camera detection | Ungraded; adds a model, a dataset and new failure modes |
| AprilTags | Still perception work that the catalog makes unnecessary |
