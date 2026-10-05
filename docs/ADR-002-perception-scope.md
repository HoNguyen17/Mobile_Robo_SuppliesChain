# ADR-002: No ML Perception; Shelf Poses Come from the Catalog

**Status:** Accepted · amended 2026-10-04 ([ADR-012](ADR-012-custom-python-navigation.md), [ADR-015](ADR-015-physical-waffle-pi-body.md))

## Context
Item detection and classification are not graded. An ML pipeline (camera, dataset, model) would take time away from avoidance tuning.

## Decision
The robot knows where every item is from the `shelf_slots` and `items` tables. The item's category is a catalog field, not something detected. Its pose (`/robot/pose`) and the static map (`/map`) are ground truth from Unity. The simulated sensors of the physical Waffle Pi ([ADR-015](ADR-015-physical-waffle-pi-body.md)) are the **LIDAR** (raycasts, `/scan`) and odometry (`/odom`). Navigation uses neither today. The LIDAR is meant only for navigation, to see obstacles that are not in the map (the scan-based layer comes later). `/odom` is not needed, because the pose is ground truth. The camera and the IMU of the real robot are not simulated.

## Consequences
- Navigation results are never mixed up with detection errors.
- Runs are deterministic and repeatable, which matters for N = 20 evidence.
- Shelf-slot poses in the database must match the Unity scene; they are exported from the scene once ([data-model.md §4](data-model.md#4-seeding)).
- The map holds only static bodies (walls, stations, racks). A dynamic box lying in an aisle is not in it, and it stays invisible to the planner until the scan-based obstacle layer exists ([ADR-012](ADR-012-custom-python-navigation.md)).

## Alternatives rejected
| Option | Why not |
|--------|---------|
| YOLO / camera detection | Ungraded; adds a model, a dataset and new failure modes |
| AprilTags | Still perception work that the catalog makes unnecessary |
