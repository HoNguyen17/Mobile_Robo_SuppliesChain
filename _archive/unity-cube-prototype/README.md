# Archived: the kinematic Cube prototype (Unity side)

Unity files of the `Nhan-turtlebot` prototype, replaced on 2026-10-04 when the robot became the physical TurtleBot3 Waffle Pi again ([ADR-015](../../docs/ADR-015-physical-waffle-pi-body.md)). They are moved here, not deleted. Unity compiles everything under `Assets/`, so they cannot stay there next to the new scripts.

| File | What it was |
|------|-------------|
| `Scripts/CubeCarNavigator.cs` | Unity-only mission for the Cube (replaced by `TurtleBotNavigator`) |
| `Scripts/CubeCmdVelSubscriber.cs` | Moved the `Cube` transform from `/cmd_vel` (replaced by `DiffDriveController` + `CmdVelSubscriber`) |
| `Scripts/CubePosePublisher.cs` | Published the Cube pose (replaced by `RobotPosePublisher`) |
| `Scripts/OccupancyGridPublisher.cs` | Built `/map` from the grid builder (replaced by `WarehouseMapPublisher`) |
| `Scripts/HelloSubscriber.cs`, `Scripts/TestPublisher.cs` | Connection smoke tests |
| `Editor/StripPhysicsFromRobot.cs` | Menu *Tools > Robot > Strip Physics (Visual Only)*, which made the URDF model a pure visual |

The `.meta` files travelled with the scripts, so moving a file back into `WarehouseProjectURP/Assets/` restores the same GUIDs.

The scene of the Cube prototype (`Warehouse.unity` with the `Cube`) is in git history: commit `47e7a67` and earlier on this branch. The ROS side of the prototype (`ros1/cube_control`, `README_ROS1_Prototype.md`) is archived separately once `ros1/turtlebot_control` passes its acceptance run.
