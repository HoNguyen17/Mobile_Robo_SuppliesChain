# Unity A* prototype (archived 2026-09-30)

`CubeCarNavigator.cs` drove a placeholder cube by editing its Transform, with an in-Unity A* planner.

Replaced by `WarehouseProjectURP/Assets/Scripts/Robot/`:
- `DiffDriveController.cs` drives the TurtleBot3 through its wheel joints.
- `TurtleBotNavigator.cs` is the same A* planner at real scale; Unity-only test mode.

The graded path uses ROS move_base instead (docs/ADR-008-navigation-stack.md).
Kept outside `Assets/` so Unity does not compile it.
