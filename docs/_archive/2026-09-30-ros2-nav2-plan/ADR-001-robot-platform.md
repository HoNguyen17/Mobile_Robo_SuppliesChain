# ADR-001: Robot Platform — TurtleBot3 Waffle Pi, No Arm

## Context
UC6 requires a mobile robot that navigates a warehouse and "retrieves" items. Grading centres on obstacle-avoidance planning, not manipulation. The team has 8 weeks and no requirement to model grasping.

## Decision
Use the TurtleBot3 Waffle Pi with no arm. "Retrieve" is modelled as: navigate to the item's known shelf pose, dwell for a configured duration, then Unity re-parents the item GameObject to the robot. No grasp planning is implemented.

## Consequences
- Manipulation, grasp planning, and arm kinematics are entirely out of scope, keeping effort on navigation.
- The Unity-side re-parenting mechanism becomes a small but load-bearing integration point (pickup/place event contract in `docs/architecture.md` §2) that must be reliable enough not to mask navigation results.
- The differential-drive footprint and published velocity/turning limits of the Waffle Pi directly set the category motion-profile baseline values in `docs/architecture.md` §6.

## Alternatives Rejected
- **TurtleBot3 Burger:** smaller footprint and lower top speed; rejected because Waffle Pi is the platform already assumed by the Unity Robotics Hub Nav2 tutorial being adapted for M1, minimising Phase 0 rework.
- **Arm-equipped platform with grasp planning:** rejected outright — manipulation is not graded and would consume effort better spent on the graded obstacle-avoidance milestones (PRD NG2).
