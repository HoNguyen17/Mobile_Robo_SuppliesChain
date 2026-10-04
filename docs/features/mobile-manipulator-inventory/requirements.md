# Feature: Mobile Manipulator + Inventory Panel

**Status:** Agreed (grilling session, 2026-10-01) · **Owner:** Nguyen · **Not started** · re-aligned 2026-10-04

This feature goes **beyond the graded plan** in [prd.md](../../prd.md). Obstacle avoidance (M1 → M3) still comes first. Nothing here may delay a phase gate.

> **Re-alignment, 2026-10-04.** The project moved to a kinematic robot body, our own Python navigation and standard ROS messages ([ADR-012](../../ADR-012-custom-python-navigation.md), [ADR-013](../../ADR-013-kinematic-robot-body.md), [ADR-014](../../ADR-014-ros-interfaces.md)). Changed here: D0 (physics standard deferred), D5 and D8, the pick flow (R4), the interfaces (§4), the phase plan (§5), the open risks (§7), and the ADR number of the manipulator (012 → **015**). The scripted arm (D3) works unchanged with a kinematic body.

---

## 1. Why

Today, picking an item means "drive to the shelf, wait (dwell), and the item jumps onto the robot" ([ADR-001](../../ADR-001-robot-platform.md)). This feature makes the warehouse look and work like a real one:

- a robot **arm** that really takes the box off the shelf and puts it down at home,
- a stable **address** for every box (rack, layer, position),
- an **in-game panel** to manage boxes, queue pick jobs and see when they finish,
- a clearly marked **HomePoint** and a **follow camera** to watch the whole process.

## 2. Decisions

| # | Topic | Decision |
|---|-------|----------|
| D1 | Priority | P0 publishers first. Cheap features in P1, the panel in P2, the arm only after the P3 gate |
| D0 | Physics | **Deferred** together with [ADR-011](../../ADR-011-physics-standard.md): the robot body is kinematic, so the physics standard is not applied. The box masses stay as data (Fragile 5, Standard 15, Heavy 30 kg) and matter only if boxes become dynamic |
| D2 | Robot | **One robot** (ADR-006 stays): TurtleBot3 Waffle Pi + OpenMANIPULATOR-X on a **lift mast** + a **tray** for one box. A two-robot "train" was considered and rejected (multi-robot navigation risk) |
| D3 | Arm | **Scripted** in Unity: preset poses (reach, grip, lift, place). The box is attached by re-parenting. No MoveIt, no physical grasping |
| D4 | Reach | Shelf layers in robot metres: layer0 = 0 m, layer1 = 0.30 m, layer2 = 0.61 m, layer3 = 0.91 m. The arm alone reaches about 0.55 m, so the **lift mast** makes all 4 layers reachable. **Fallback:** scale the robot up until it reaches every layer and the box fits on the tray |
| D5 | Reach test | 1–2 day reach test in **P1**. It decides the final robot scale. A later scale change stays cheap: Unity rebuilds the map at every Play and ROS sees robot metres ([ADR-010](../../ADR-010-robot-scale.md)) |
| D6 | Panel | **In-game Unity panel** now. A web inventory dashboard (React + TS via rosbridge) is a future item |
| D7 | Shelves | Shelves are **fixed** (the static map depends on them). The panel manages **boxes** only |
| D8 | Data | Inventory lives in **MySQL**, reached through **ROS topics with acknowledgements** ([ADR-014](../../ADR-014-ros-interfaces.md)). Only the `task_manager` class touches MySQL ([ADR-005](../../ADR-005-mysql-database.md)). The panel is read-only when ROS is off |
| D9 | First fill | At first start the 144 slots are filled **randomly once with a fixed seed** and saved to the DB. Seed files (exact box per slot) can be loaded for test scenarios. The warehouse's own random box re-spawn (`ShelfBoxRandomizerShim`) is **turned off** so IDs never change |
| D10 | Trip | **One box per trip** |
| D11 | Drop-off | A small **table on the wall edge of the HomePoint tile** holds **2** boxes; when a third arrives, the oldest disappears. The DB keeps a `delivered` record for every box |

## 3. Requirements

### R1. Addresses and IDs

| ID | Requirement |
|----|-------------|
| R1.1 | Racks are numbered **`R01`–`R12`**, starting from the HomePoint corner, column by column: `R01–R04` in the column at x = -12.5 (z = 15 → -15), `R05–R08` at x = 0, `R09–R12` at x = 12.5 |
| R1.2 | Slot names copy Unity's prefab names exactly (0-based): **`R03-layer2-pos1`**. Each rack has `layer0`–`layer3` × `pos0`–`pos2` = 12 slots; **144 slots** in total |
| R1.3 | Boxes have IDs **`B-0001`, `B-0002`, …** assigned by the DB |
| R1.4 | Every rack shows a floating label with its ID in Unity. Labels can be toggled with a key |
| R1.5 | A box's full address is shown as: `B-0007 · R03-layer2-pos1 · Column 1, Row 3 · (x 3.1 m, y -1.2 m)`. The position is in map metres |

### R2. Boxes

| ID | Requirement |
|----|-------------|
| R2.1 | A box has: **model** (one of the warehouse's existing box prefabs), **category** (Fragile / Standard / Heavy), optional **name** (e.g. "Bolts M8") |
| R2.2 | Each box shows a small **coloured sticker** for its category |
| R2.3 | On Play, Unity spawns the boxes from the DB into their slots |
| R2.4 | The category still selects the motion profile of the drop-off leg (existing plan) |

### R3. In-game panel (Unity, toggle with `Tab`)

| ID | Requirement |
|----|-------------|
| R3.1 | List all boxes with ID, name, category and address. Filter by rack and by category |
| R3.2 | **Add** a box to an empty slot, **edit** its name/category/model, **delete** it |
| R3.3 | Selecting a box **outlines the box and its rack** in the 3D view |
| R3.4 | Select **several boxes** and press **Start**: they are queued and run one after another (FIFO) |
| R3.5 | Each task shows its state: `queued → to shelf → picking → to home → placing → done / failed` |
| R3.6 | Buttons: **Start**, **Pause queue**, **Cancel current task** (the robot drives back home) |
| R3.7 | When a task ends, a pop-up message appears, e.g. "B-0007 delivered in 1 m 42 s, 0 collisions", and the task is added to a **history** list |
| R3.8 | When ROS is not connected, the panel shows the last loaded list as **read-only**, with a clear "ROS offline" banner |

### R4. Pick flow (one task)

1. The robot drives to the box's slot (shelf leg, planner and follower).
2. The lift mast raises the arm to the slot's layer. The arm takes the box onto the tray.
3. The robot drives to HomePoint (drop-off leg, with the box's category profile).
4. The arm places the box on the drop-off table (max 2 boxes; the oldest disappears).
5. The DB marks the box `delivered`. The next queued task starts, or the robot stays idle at HomePoint.

### R5. HomePoint

| ID | Requirement |
|----|-------------|
| R5.1 | HomePoint is on the floor tile `Floor01(Clone)` at **(-23.48, 0, 25.29)** (Unity world), in the corner of the warehouse, clear of all racks |
| R5.2 | It is marked with a **glowing coloured square** on the floor |
| R5.3 | It is the robot's park position at start and end, and the unload station |

### R6. Camera 2

| ID | Requirement |
|----|-------------|
| R6.1 | A **chase camera** follows behind the robot |
| R6.2 | It is shown **picture-in-picture** in a corner by default |
| R6.3 | Key **`C`** swaps which camera is full-screen |

## 4. New interfaces (to design in detail later)

There are no ROS services any more ([ADR-014](../../ADR-014-ros-interfaces.md)). Each call below becomes a command topic plus an acknowledgement, the same pattern as the `/sim/*` topics in [architecture.md §5](../../architecture.md#5-interface-contract).

| Interface | Kind | Purpose |
|-----------|------|---------|
| `/inventory/list_boxes`, `/inventory/add_box`, `/inventory/update_box`, `/inventory/delete_box` | Command topics + acknowledgement, handled by `task_manager` | Panel ↔ MySQL |
| `/tasks/enqueue`, `/tasks/pause`, `/tasks/cancel` | Command topics + acknowledgement | Panel → mission queue |
| `/tasks/status` | ROS topic | Task states for the panel |
| `/sim/pick_item`, `/sim/place_item` | Command topics + `/sim/ack`, handled by Unity | Replace `attach_item` / `release_item` once the arm exists |
| `slots`, `boxes` tables | MySQL | Extends `shelf_slots` / `items` in [schema.sql](../../schema.sql) |

## 5. Phase plan

| When | Work |
|------|------|
| **P0** (now) | The prototype's `/cube/pose`, `/map` and `/cmd_vel` scripts already exist; P1 adds the clock, the scale and the `/sim/*` topics; `warehouse_bringup` is adapted (not part of this feature, but everything needs it) |
| **P1** | R1 IDs + labels · R2 boxes spawned from DB, randomizer off · R5 HomePoint marker · R6 camera 2 · **D5 reach test** |
| **P2** | R3 in-game panel: inventory, queue, notifications |
| **After P3 gate** | **ADR-015** (mobile manipulator, replaces ADR-001) · arm + lift mast + tray + drop-off table · R4 pick flow |
| **Future** | Web inventory dashboard (React + TS, rosbridge) |

## 6. Docs to update when this is built

- New **ADR-015**: mobile manipulator; replaces ADR-001; updates PRD non-goal NG2. (ADR-012 to ADR-014 are taken by the 2026-10-04 re-alignment.)
- [prd.md](../../prd.md): move "grasp with an arm" out of the non-goals.
- [architecture.md](../../architecture.md): new topics (§4 above).
- [data-model.md](../../data-model.md) + [schema.sql](../../schema.sql): box and slot tables.
- [CONTEXT.md](../../../CONTEXT.md): new words: **Rack**, **Slot address**, **Lift mast**, **Tray**, **Drop-off table**, **Panel**, **Queue**. "Attach" becomes "pick" once the arm exists.

## 7. Open risks

| Risk | Plan |
|------|------|
| Lift mast + arm upsets balance or physics | No longer a risk: the body is kinematic ([ADR-013](../../ADR-013-kinematic-robot-body.md)). If a physical body returns, re-run the drive tests with the extra mass |
| Robot with mast does not fit between racks | Check footprint vs aisle width in the reach test |
| A bigger robot (fallback) changes the footprint | The numbers stay in robot metres; only the footprint and `inflation_radius` need a re-check. Decide in P1; update ADR-010 and test-plan |
| Feature work steals time from M2/M3 | Nothing in P2+ starts before that phase's gate passes |
