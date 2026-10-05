# ADR-013: Robot Body = Kinematic Cube with a Waffle Pi Visual Model

**Status:** Superseded by [ADR-015](../../ADR-015-physical-waffle-pi-body.md) (2026-10-04) · amends [ADR-001](../../ADR-001-robot-platform.md), [ADR-009](../../ADR-009-simulation-clock.md) and [ADR-010](../../ADR-010-robot-scale.md) · defers [ADR-011](../../ADR-011-physics-standard.md)

## Context
Branch `Nguyen-planning` built a physical Waffle Pi: articulated wheels, scale 4, a physics standard, Edit Mode and Play Mode tests. The `Nhan-turtlebot` prototype uses a simpler body: the Unity object `Cube`, moved through its transform, with a TurtleBot3 model attached for looks only. The team code base is the second one ([ADR-012](../../ADR-012-custom-python-navigation.md)).

## Decision
1. **Body.** The Unity object `Cube` is moved by `CubeCmdVelSubscriber` through its transform. There is no physics, no wheel dynamics (the wheels do not turn) and no collision response. Its 1 x 1 x 1 box collider is the footprint.
2. **Visual.** `turtlebot3_waffle_pi` is a child of the `Cube`, scale **3.2**, position (0, -0.5, 0). Its physics components are removed with *Tools > Robot > Strip Physics (Visual Only)* (`Assets/Editor/StripPhysicsFromRobot.cs`). Step by step: [README_ROS1_Prototype.md](../../../_archive/ros1-cube-prototype/README_ROS1_Prototype.md).
3. **Scale and units.** ROS sees robot metres. Positions, the map origin and cell size, and scan ranges are divided by 3.2 on the way out; the linear part of `/cmd_vel` is multiplied by 3.2 on the way in. The scale lives in one place, the Transform of the visual model (the `Cube` itself stays at scale 1). The half-diagonal of the `Cube` footprint (0.71 Unity units) is about 0.22 m in robot metres, close to the 0.21 m footprint radius of the Waffle Pi. The grid builder's own `Robot Radius` is set to 0, because the planner inflates the map ([ADR-012](../../ADR-012-custom-python-navigation.md)). All numbers in [test-plan.md](../../test-plan.md) are in robot metres.
4. **Sensors, all emulated.** The pose is ground truth (`PoseStamped`, stamped with simulation time). The LIDAR is raycasts, LDS-01-like: 360 beams, 0.12 to 3.5 m in robot metres, 5 Hz, on `/scan`. It is ported from `Nguyen-planning` in P2 (`LaserScanner`, `LaserScanPublisher`, `PublishTimer`). There is no odometry.
5. **Collisions.** `CollisionReporter` on the `Cube` (trigger collider + kinematic Rigidbody). Overlapping a wall, shelf, box or NPC collider is a collision. It is debounced to 1 s per object and reported on `/sim/collision`.
6. **Clock.** Unity publishes `/clock` ([ADR-009](../../ADR-009-simulation-clock.md)).

## Consequences
- Stable and exact: the robot never tips, and it moves at exactly the commanded speed, which keeps the mean-speed metric of S-05 clean.
- The `Cube` passes through anything. Collisions are **reported, not prevented**. Inflation and the obstacle layer ([ADR-012](../../ADR-012-custom-python-navigation.md)) are the only protection.
- The three ROS scripts of the prototype (`CubePosePublisher`, `CubeCmdVelSubscriber`, `OccupancyGridPublisher`) must apply the scale and the new message types: P1 task.
- `Nguyen-planning` keeps `DiffDriveController`, `WheelHold`, the physics standard and their tests. A physical body later needs wheel-level control, a verified scale, `/odom` and a re-run of the ADR-010 checks (the "Next steps" of the prototype README).
- The scripted arm of the [manipulator feature](../../features/mobile-manipulator-inventory/requirements.md) works with a kinematic body, and its "mast upsets balance" risk disappears.
- Unity rebuilds the map at every Play and ROS sees robot metres, so changing the scale after the reach test (feature D5) does not invalidate a saved map.

## Alternatives rejected
| Option | Why not |
|--------|---------|
| Physical Waffle Pi (ADR-010 / ADR-011) | Verified on `Nguyen-planning`, but it needs `/odom`, TF and tuned physics. Deferred, not deleted |
| Plain cube without the Waffle model | Works, but the model makes the demo readable and costs no physics |
