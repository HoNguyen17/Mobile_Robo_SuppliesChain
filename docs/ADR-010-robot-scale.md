# ADR-010: Robot Scaled 4x in Unity, with Consistent Physics

**Status:** Accepted

## Context
The imported warehouse is large relative to a real-size TurtleBot3 (0.28 m wide). At scale 1 the robot looks tiny next to the shelves.

## Decision
The robot root (`turtlebot3_waffle_pi`) is scaled **4, 4, 4** in Unity.

- **Robot metres vs Unity units:** 1 robot metre = 4 Unity units. All robot dimensions in code (wheel radius, wheel separation, speeds, safety radius) stay in **robot metres**, and scripts multiply by the robot's scale where they touch the world.
- **ROS sees robot metres.** `OdometryPublisher` and `LaserScanPublisher` must **divide positions and ranges by the scale (4)** before publishing, and `/cmd_vel` is already in robot m/s. Then the ROS map, footprint, `move_base` limits and all numbers in [test-plan.md](test-plan.md) stay in true metres, whatever the Unity scale is.
- **Scenario sizes** in test-plan (corridor width, box size) are in robot metres. The Unity scenes must build them at 4x.
- The scale lives in one place, the robot root's Transform. Scripts read `transform.lossyScale.x`.

## Physics setup (all in `DiffDriveController`, found and verified by Play Mode tests)
A straight URDF import does **not** drive in Unity. Five separate problems had to be fixed:

| Problem found | Symptom | Fix |
|---------------|---------|-----|
| The articulation root is `base_link`, not `base_footprint` (which is a plain transform) | Anything reading `base_footprint` sees the robot as never moving | `DiffDriveController.BaseBody` returns the real moving body; the navigator uses it |
| Warehouse floor `BoxCollider`s are **zero thickness** (`Floor01.prefab`, height 1e-13) | Robot parts sink 1-2 cm through the floor and are never pushed out, so the robot leans on its wheels and cannot balance | `FloorColliderFix.Apply()` gives thin floor slabs 0.2 m of thickness, top surface unchanged |
| Importer colliders: chassis/casters lower than the wheels (scale 1) or dug into the floor (scale 4) | Wheels spin without traction, or jam | Wheel colliders become spheres, casters become frictionless balls at wheel-bottom height, chassis boxes are lifted clear |
| Links without `<inertial>` (camera, imu) get Unity's default **1 kg each** | ~4 kg of phantom weight ahead of the axle tips the 1.4 kg robot forward | Those links are set to 0.001 kg |
| Unity scales colliders with the transform, but **not** mass, centre of mass or inertia | A 4x robot has 1/64 of the mass it should | mass x s^3, centre of mass x s, inertia x s^5, wheel torque x s^5 |

Smaller decisions: speed ramps with an acceleration limit (1 m/s^2 linear, 3 rad/s^2 angular) so the robot never wheelies; friction of "slick" parts uses `Multiply` (Unity's `Minimum` loses to the floor's default `Average`); the wheel joint speed cap is raised from the importer's 7 rad/s to 100.

## Verification (Play Mode tests, `Assets/Tests/PlayMode/`)
Run with **Window > General > Test Runner > PlayMode**. On the real `Warehouse` scene at scale 4:

| Check | Result |
|-------|--------|
| Settle under gravity | level (`up.y = 1.00`), all contacts at y = 0.000 |
| Forward, 0.2 m/s for 5 s | 0.95 m (expected ~0.98 after the speed ramp) |
| Turn left, 1 rad/s for 3 s | -156 deg (expected -172 deg after the ramp), no drift |
| Navigator mission (start -> shelf -> dwell -> home) | shelf reached at 38 s, home reached at 66 s, stops 0.6 m from `HomePoint`, never tips |

## Consequences
- Do not change the scale after building the ROS map, or the map and the world will disagree.
- **Only scale 4 is verified.** At scale 1 the robot drives straight and balances but pivots weakly (~20 degrees instead of 172). If a different scale is ever needed, re-run the tests and tune `DiffDriveController` before trusting it.
- Unity-only test mode (`TurtleBotNavigator`) already handles the scale. Keyboard test: with Keyboard Teleop ticked, up = forward and left = turn left with **no** wheel-invert boxes ticked.
- Any other dynamic object added to the warehouse benefits from the thick floors too.
