# Diagrams — UC6 Mobile Robot Warehouse System

All diagrams are Mermaid, inline, per NFR-05. No external diagramming tool output is used.

## (a) End-to-End Task Workflow (Flowchart)

```mermaid
flowchart TD
    A[Episode start] --> B[task_manager reads next pending task]
    B -->|MySQL reachable| C[Task returned to mission_orchestrator]
    B -->|MySQL unreachable| B2[Fallback: cached/default task, log warning]
    B2 --> C
    C --> D[navigate_to_pose: shelf slot]
    D --> E{Static obstacle?}
    E -->|Yes| F[Local planner avoids / global re-plan]
    E -->|No| G[Continue tracking global path]
    F --> G
    G --> H{Arrived at shelf pose?}
    H -->|No, stuck| I[Recovery behaviours]
    I --> D
    H -->|Yes| J[Dwell N seconds]
    J --> K[Unity re-parents item to robot]
    K --> L[motion_profile_node applies category profile]
    L --> M[navigate_to_pose: drop-off]
    M --> N{Dynamic obstacle?}
    N -->|Yes| O[Local avoidance / re-plan, log replan_event]
    N -->|No| P[Continue tracking global path]
    O --> P
    P --> Q{Arrived at drop-off pose?}
    Q -->|No, stuck| R[Recovery behaviours]
    R --> M
    Q -->|Yes| S[Dwell N seconds]
    S --> T[Unity releases item]
    T --> U[metrics_collector finalises run summary]
    U --> V[task_manager writes runs + run_events]
    V -->|MySQL unreachable| V2[Buffer to disk, retry next episode]
    V --> W[Episode end]
    V2 --> W
```

## (b) Sequence Diagram — Task Dispatch to Report

```mermaid
sequenceDiagram
    participant DB as MySQL
    participant TM as task_manager
    participant MO as mission_orchestrator
    participant MP as motion_profile_node
    participant NAV as Nav2 (bt_navigator/planner/controller)
    participant UB as Unity Bridge
    participant MC as metrics_collector

    TM->>DB: SELECT next pending task
    DB-->>TM: task row (or fallback on failure)
    TM->>MO: get_next_task response
    MO->>NAV: navigate_to_pose(shelf_pose)
    NAV->>UB: cmd_vel
    UB-->>NAV: odom / scan / tf
    NAV-->>MC: replan_event (if triggered)
    NAV-->>MO: result: SUCCEEDED
    MO->>UB: PickupEvent (dwell complete)
    UB-->>MO: PickupAck (re-parent done)
    MO->>MP: set_category(item.category)
    MP->>NAV: param set (max_vel_x, acc_lim_x, inflation_radius)
    MO->>NAV: navigate_to_pose(dropoff_pose)
    NAV->>UB: cmd_vel
    UB-->>NAV: odom / scan / tf
    UB-->>MC: collision_event (if any)
    NAV-->>MO: result: SUCCEEDED
    MO->>UB: PlaceEvent (dwell complete)
    UB-->>MO: PlaceAck (un-parent done)
    MO->>MC: episode outcome
    MC->>TM: run summary (aggregated events)
    TM->>DB: INSERT runs, run_events (single transaction)
```

## (c) Robot State Machine

```mermaid
stateDiagram-v2
    [*] --> Idle
    Idle --> NavigatingToShelf: task received
    NavigatingToShelf --> Avoiding: obstacle detected
    Avoiding --> NavigatingToShelf: path clear / re-planned
    NavigatingToShelf --> Recovering: controller stuck / plan failed
    Recovering --> NavigatingToShelf: recovery succeeded
    Recovering --> Aborted: recovery exhausted
    NavigatingToShelf --> DwellingAtShelf: goal reached
    DwellingAtShelf --> ProfileApplied: dwell complete, item re-parented
    ProfileApplied --> NavigatingToDropoff: category profile applied
    NavigatingToDropoff --> Avoiding: obstacle detected
    NavigatingToDropoff --> Recovering: controller stuck / plan failed
    NavigatingToDropoff --> DwellingAtDropoff: goal reached
    DwellingAtDropoff --> Completed: dwell complete, item released
    Completed --> [*]
    Aborted --> [*]
```

## (d) Entity-Relationship Diagram

```mermaid
erDiagram
    CATEGORIES ||--o{ ITEMS : "classifies"
    SHELF_SLOTS ||--o| ITEMS : "may hold"
    ITEMS ||--o{ TASKS : "requested in"
    TASKS ||--o{ RUNS : "attempted as"
    RUNS ||--o{ RUN_EVENTS : "logs"

    CATEGORIES {
        int category_id PK
        varchar name
        decimal max_vel_x
        decimal acc_lim_x
        decimal inflation_radius
    }
    SHELF_SLOTS {
        int slot_id PK
        varchar aisle
        decimal pose_x
        decimal pose_y
        decimal pose_theta
        int occupied_item_id FK
    }
    ITEMS {
        int item_id PK
        varchar name
        int category_id FK
        int home_slot_id FK
        decimal weight_kg
        timestamp created_at
    }
    TASKS {
        int task_id PK
        int item_id FK
        decimal dropoff_x
        decimal dropoff_y
        decimal dropoff_theta
        tinyint priority
        enum status
        timestamp created_at
    }
    RUNS {
        int run_id PK
        int task_id FK
        varchar scenario_id
        varchar robot_namespace
        enum outcome
        int collisions_count
        decimal min_clearance_m
        decimal path_length_m
        decimal straight_line_baseline_m
        decimal path_length_ratio
        decimal time_to_goal_s
        int replan_count
        decimal mean_linear_vel_mps
        timestamp start_time
        timestamp end_time
    }
    RUN_EVENTS {
        bigint event_id PK
        int run_id FK
        enum event_type
        timestamp event_time
        json payload_json
    }
```

## (e) Deployment / Process View

```mermaid
flowchart TB
    subgraph Host Machine
        subgraph Unity Process
            UP[Unity Editor / Player - Play or batch mode]
        end
        subgraph ROS 2 Workspace - one per robot namespace
            RE[ros_tcp_endpoint]
            NAV2[Nav2 stack: bt_navigator, planner_server, controller_server, behavior_server, costmaps]
            LOC[slam_toolbox / amcl]
            APP[mission_orchestrator, motion_profile_node, metrics_collector]
        end
        subgraph Docker Compose Network
            MYSQL[(MySQL container)]
        end
        TASKM[task_manager]
    end

    UP <-- TCP --> RE
    RE --- NAV2
    RE --- LOC
    APP --- NAV2
    TASKM <--> MYSQL
    TASKM --- APP

    note1[Stage B/C: repeat ROS 2 Workspace subgraph per additional robot, namespaced robot2/, robot3/; one ros_tcp_endpoint per robot unless multiplexing is verified - see docs/architecture.md §7]
```
