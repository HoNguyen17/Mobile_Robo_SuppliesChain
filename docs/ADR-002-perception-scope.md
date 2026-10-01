# ADR-002: No ML Perception; Shelf Poses Come from the Catalog

**Status:** Accepted

## Context
Item detection and classification are not graded. An ML pipeline (camera, dataset, model) would take time away from avoidance tuning.

## Decision
The robot knows where every item is from the `shelf_slots` and `items` tables. The item's category is a catalog field, not something detected. The only sensor used for decisions is the **LIDAR**, and only for navigation.

## Consequences
- Navigation results are never mixed up with detection errors.
- Runs are deterministic and repeatable, which matters for N = 20 evidence.
- Shelf-slot poses in the database must match the Unity scene; they are exported from the scene once ([data-model.md §4](data-model.md#4-seeding)).

## Alternatives rejected
| Option | Why not |
|--------|---------|
| YOLO / camera detection | Ungraded; adds a model, a dataset and new failure modes |
| AprilTags | Still perception work that the catalog makes unnecessary |
