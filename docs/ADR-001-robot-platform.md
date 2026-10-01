# ADR-001: TurtleBot3 Waffle Pi, No Arm

**Status:** Accepted

## Context
The robot must "retrieve" items, but the grade is for obstacle avoidance, not manipulation.

## Decision
Use a **TurtleBot3 Waffle Pi** without an arm. Retrieval works like this:

1. Drive to the item's known shelf pose.
2. Dwell `dwell_s` seconds.
3. Unity re-parents the item to the robot (`/sim/attach_item`).

Release at drop-off is the reverse (`/sim/release_item`).

## Consequences
- No grasping, arm kinematics or MoveIt.
- The URDF, meshes and navigation configs come ready-made from the `ros-noetic-turtlebot3*` packages.
- The Waffle Pi's 0.26 m/s top speed sets the Standard motion profile.

## Alternatives rejected
| Option | Why not |
|--------|---------|
| TurtleBot3 Burger | Smaller, but the Waffle Pi's reference configs match what we copy from |
| Robot with an arm | Ungraded work that costs weeks |
