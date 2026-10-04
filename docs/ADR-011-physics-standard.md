# ADR-011: Physics Standard (Exact 4x Model of Earth)

**Status:** Accepted (2026-10-01)

## Context
The robot is scaled 4x in Unity ([ADR-010](ADR-010-robot-scale.md)), and the warehouse matches it: in real metres (what ROS sees), it is a small warehouse with 0.30 m shelf layers and 0.28 m boxes. Before this ADR, physics values were whatever the imported assets happened to contain:

| Before | Problem |
|--------|---------|
| Gravity 9.81 m/s² in Unity | Lengths are x4 but time is not, so in real units gravity was **4x too weak**: less wheel grip, objects fall 2x too slowly, tipping is too hard |
| Boxes 2 kg / 5 kg for a 1.13 m box | Real density 95–237 kg/m³ by chance; no rule behind it |
| Rack legs: 500 kg rigidbodies, gravity off, all axes frozen | Made-up mass for something that should simply be static |
| No physic materials (Unity default friction everywhere) | Friction undefined |

We are going to add boxes, an arm and moving NPCs. Each of them needs physics that follows one rule, and that rule must still hold when an object is scaled.

## Decision
The Unity scene is an **exact 4x model of a small real warehouse on Earth**. All physics follows from one table of scale rules, implemented once in `PhysicsStandard` (`Assets/Scripts/Physics/`).

| Quantity | Real → Unity | Why |
|----------|--------------|-----|
| Length | x4 | `WorldScale = 4` (= robot root scale) |
| Time | x1 | ROS and Unity share one clock ([ADR-009](ADR-009-simulation-clock.md)) |
| Speed | x4 | length / time |
| Acceleration, **gravity** | x4 → **39.24 m/s²** | length / time² |
| Mass | x64 | volume, same densities |
| Force | x256 | mass × acceleration |
| Torque, inertia | x1024 | force × length, mass × length². `DiffDriveController` already used s⁵ |

With these rules, anything that happens in Unity is exactly what would happen on Earth, read in real metres.

### Authors only type real values
- **`PhysicalBody`** component: surface material, static or dynamic, **real mass in kg**. It derives the Unity mass (`real × 64 × scale volume`), gravity flags and friction. Scaling an object scales its mass with its volume automatically.
- It owns every collider and rigidbody below it, except those under a deeper `PhysicalBody`. So a static rack holds dynamic boxes.
- Gravity is set in the project settings and again at startup (`PhysicsStandard.ApplyGravity`), so it is right in builds and tests too.
- The robot does not use `PhysicalBody`. `DiffDriveController` scales the URDF masses by the same rule (x s³, inertia x s⁵). The robot's wheel and caster materials stay as verified in ADR-010.

### Surface materials (friction combines by **Average**, no bounce)

| Surface | Static | Dynamic | Used for |
|---------|:------:|:-------:|----------|
| Concrete | 0.8 | 0.7 | Floor, walls, ceiling |
| Steel | 0.5 | 0.4 | Racks |
| Cardboard | 0.5 | 0.4 | Boxes |
| Rubber | 1.0 | 0.9 | Grippers, future |
| Plastic | 0.4 | 0.3 | Stations, trays |

### Real masses

| Object | Real | Unity |
|--------|-----:|------:|
| Box, Fragile | 5 kg | 320 kg |
| Box, Standard | 15 kg | 960 kg |
| Box, Heavy | 30 kg | 1 920 kg |
| TurtleBot3 Waffle Pi (URDF links) | 1.55 kg | 99 kg |

The Heavy box equals the Waffle Pi's maximum payload (30 kg, ROBOTIS spec). Box size stays 1.13 m in Unity (0.28 m real, real density 237 / 710 / 1 420 kg/m³).

## Applying and checking it
1. **Robotics → Warehouse → Apply Physics Standard** sets the gravity and adds a `PhysicalBody` to the shell (static concrete), stations (static plastic), 12 racks (static steel) and 81 boxes (dynamic cardboard, Standard). Then save the scene.
2. EditMode tests `PhysicsStandardTests`, `PhysicalBodyTests` and `WarehousePhysicsSceneTests` check the rules. They also check that every physics object in `Warehouse.unity` has a `PhysicalBody`, that the 12 racks are static steel, and that real densities are plausible (20–3000 kg/m³).

## Consequences
- Every new physics object needs a `PhysicalBody` (the scene test fails otherwise). For boxes, use the category mass.
- Things fall 4x faster in Unity units than they look like they should. That is correct: in real metres they fall like on Earth.
- The ADR-010 PlayMode results were measured with g = 9.81 and **must be re-run**. More grip should only help, but tuning may change.
- `WorldScale` and the robot root scale must stay equal. The scene test checks this.
