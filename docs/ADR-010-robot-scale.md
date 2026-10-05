# ADR-010: Robot Scaled 4x in Unity, ROS Sees Robot Metres

**Status:** Accepted 2026-10-02 (physical body, scale 4); re-activated 2026-10-04 by [ADR-015](ADR-015-physical-waffle-pi-body.md) after a short kinematic-Cube period (scale 3.2, [ADR-013](_archive/2026-10-04-kinematic-cube-plan/ADR-013-kinematic-robot-body.md), see History)

## Context
The imported warehouse is large relative to a real-size TurtleBot3 (0.28 m wide). At scale 1 the robot looks tiny next to the shelves.

## Decision
The robot root (`turtlebot3_waffle_pi`) is scaled **4, 4, 4** in Unity.

- **Robot metres vs Unity units:** 1 robot metre = 4 Unity units. All robot dimensions in code (wheel radius, wheel separation, speeds, safety radius) stay in **robot metres**, and scripts multiply by the robot's scale where they touch the world.
- **ROS sees robot metres.** The Unity publishers (`RobotPosePublisher`, `OdometryPublisher`, `LaserScanPublisher`, `WarehouseMapPublisher`, `CollisionReporter`) divide positions, the map origin and cell size, and scan ranges by the scale before publishing. `/cmd_vel` is in robot m/s and `DiffDriveController` applies the scale on the way in. Then the ROS map, footprint, planner limits and all numbers in [test-plan.md](test-plan.md) stay in true metres, whatever the Unity scale is.
- **Scenario sizes** in test-plan (corridor width, box size) are in robot metres. The Unity scenes must build them at 4x.
- The scale lives in one place, the robot root's Transform. Scripts read `transform.lossyScale.x`. `PhysicsStandard.WorldScale` ([ADR-011](ADR-011-physics-standard.md)) must equal it; the scene test checks this.

### Physics setup (all in `DiffDriveController`, found and verified by Play Mode tests)
A straight URDF import does **not** drive in Unity. Five separate problems had to be fixed:

| Problem found | Symptom | Fix |
|---------------|---------|-----|
| The articulation root is `base_link`, not `base_footprint` (which is a plain transform) | Anything reading `base_footprint` sees the robot as never moving | `DiffDriveController.BaseBody` returns the real moving body; the navigator uses it |
| Warehouse floor `BoxCollider`s are **zero thickness** (`Floor01.prefab`, height 1e-13) | Robot parts sink 1-2 cm through the floor and are never pushed out, so the robot leans on its wheels and cannot balance | `FloorColliderFix.Apply()` gives thin floor slabs 0.2 m of thickness, top surface unchanged |
| Importer colliders: chassis/casters lower than the wheels (scale 1) or dug into the floor (scale 4) | Wheels spin without traction, or jam | Wheel colliders become spheres, casters become frictionless balls at wheel-bottom height, chassis boxes are lifted clear |
| Links without `<inertial>` (camera, imu) get Unity's default **1 kg each** | ~4 kg of phantom weight ahead of the axle tips the 1.4 kg robot forward | Those links are set to 0.001 kg |
| Unity scales colliders with the transform, but **not** mass, centre of mass or inertia | A 4x robot has 1/64 of the mass it should | mass x s^3, centre of mass x s, inertia x s^5, wheel torque x s^5 |

Smaller decisions: speed ramps with an acceleration limit (1 m/s^2 linear, 3 rad/s^2 angular) so the robot never wheelies; friction of "slick" parts uses `Multiply` (Unity's `Minimum` loses to the floor's default `Average`); the wheel joint speed cap is raised from the importer's 7 rad/s to 100. **Holding brake** (added with ADR-011): once the command is zero and the speed ramp has reached zero, each wheel drive switches from pure velocity control to a position spring that reaches full motor torque at 2° (`WheelHold`, `holdWhenStopped`), like the holding torque of the real Dynamixel servos. Without it the idle robot crept about 1.6 mm/s (real) under the stronger gravity. A new command releases it immediately.

## Verification (Play Mode tests, `Assets/Tests/PlayMode/`)
Run with **Window > General > Test Runner > PlayMode**. Measured on branch `Nguyen-planning`, on the real `Warehouse` scene at scale 4, re-run on 2026-10-02 with the ADR-011 gravity (39.24 m/s² in Unity) and the holding brake:

| Check | Result |
|-------|--------|
| Settle under gravity | level (`up.y = 1.00`), all contacts at y = 0.000 |
| Forward, 0.2 m/s for 5 s | 0.94 m (expected ~0.98 after the speed ramp) |
| Turn left, 1 rad/s for 3 s | -152 deg (expected -172 deg after the ramp), drift 0.003 m |
| Navigator mission (start -> shelf -> dwell -> home) | shelf reached at 46 s, home reached at 108 s, stops 0.2 m (real) from `HomePoint`, never tips |
| Idle 30 s with no command (holding brake) | 0.00 mm drift; then drives 0.35 m in 2 s at 0.2 m/s |

At g = 9.81 (before ADR-011) the same checks gave 0.95 m forward and -156 deg turn.

The tests and scripts came back into this branch with ADR-015 (`RobotNavigatorTests` now calls `GoToTarget()` itself). The Edit Mode and Play Mode tests were re-run in the Unity Editor on 2026-10-05 and are green (reported by the user, no figures). The table above stays the last measured state (2026-10-02); this branch has no new figures.

## Consequences
- Do not change the scale after building the ROS map, or the map and the world will disagree. Unity rebuilds `/map` from the static colliders at every Play, so the map in use is always the current one.
- **Only scale 4 is verified.** At scale 1 the robot drives straight and balances but pivots weakly (~20 degrees instead of 172). If a different scale is ever needed, re-run the tests and tune `DiffDriveController` before trusting it.
- Unity-only test mode (`TurtleBotNavigator`) already handles the scale. Keyboard test: with Keyboard Teleop ticked, up = forward and left = turn left with **no** wheel-invert boxes ticked.
- Any other dynamic object added to the warehouse benefits from the thick floors too.

---

## History: kinematic Cube at scale 3.2 (2026-10-04, superseded by ADR-015)

> On 2026-10-04 this ADR was first amended by [ADR-013](_archive/2026-10-04-kinematic-cube-plan/ADR-013-kinematic-robot-body.md): scale 3.2 instead of 4, on a kinematic `Cube` body, with the physics part deferred. ADR-015 dropped the `Cube`, so none of the text below applies any more. It is kept for the record.

- The visual model of the robot (`turtlebot3_waffle_pi`, a child of `Cube`) was scaled **3.2, 3.2, 3.2**; 1 robot metre = 3.2 Unity units. ROS saw robot metres: the Unity scripts divided positions, the map origin and cell size, and scan ranges by 3.2 and multiplied the linear part of `/cmd_vel` by 3.2. Scenario sizes in test-plan were built at 3.2x.
- The scale lived in the Transform of the visual model; the `Cube` itself stayed at scale 1, because its 1 m box was the footprint.
- Because that body was kinematic, any scale was possible. 3.2 was chosen because the model then fit the 1 m footprint of the `Cube`, whose half-diagonal is about 0.22 m in robot metres.
- Unity rebuilt the map at every Play, so a scale change would not have invalidated a saved map.
- The three ROS scripts of that prototype (`CubePosePublisher`, `CubeCmdVelSubscriber`, `OccupancyGridPublisher`, now in [`_archive/unity-cube-prototype/`](../_archive/unity-cube-prototype/README.md) at the repo root) did not apply the scale yet; that was a P1 task of the Cube plan.
