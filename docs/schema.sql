-- UC6 Mobile Robot Warehouse System — MySQL schema
-- Applied via docker-compose init volume (/docker-entrypoint-initdb.d/).
-- Exactly one ROS 2 node (task_manager) reads/writes against this schema at runtime.
-- Engine/charset chosen for FK + JSON support on a standard MySQL 8.x image. [VERIFY exact image tag in Phase 0.]

SET NAMES utf8mb4;
SET FOREIGN_KEY_CHECKS = 0;

-- ---------------------------------------------------------------------------
-- categories: fixed motion-profile parameter sets (see docs/architecture.md §6)
-- ---------------------------------------------------------------------------
CREATE TABLE IF NOT EXISTS categories (
    category_id     INT UNSIGNED    NOT NULL AUTO_INCREMENT,
    name            VARCHAR(32)     NOT NULL,
    max_vel_x       DECIMAL(4,2)    NOT NULL COMMENT 'm/s',
    acc_lim_x       DECIMAL(4,2)    NOT NULL COMMENT 'm/s^2',
    inflation_radius DECIMAL(4,2)   NOT NULL COMMENT 'metres',
    PRIMARY KEY (category_id),
    UNIQUE KEY uq_categories_name (name)
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4;

-- ---------------------------------------------------------------------------
-- shelf_slots: known static shelf poses (stand in for perception; ADR-002)
-- ---------------------------------------------------------------------------
CREATE TABLE IF NOT EXISTS shelf_slots (
    slot_id         INT UNSIGNED    NOT NULL AUTO_INCREMENT,
    aisle           VARCHAR(16)     NOT NULL,
    pose_x          DECIMAL(8,3)    NOT NULL COMMENT 'metres, warehouse frame',
    pose_y          DECIMAL(8,3)    NOT NULL COMMENT 'metres, warehouse frame',
    pose_theta      DECIMAL(6,4)    NOT NULL COMMENT 'radians',
    occupied_item_id INT UNSIGNED   NULL,
    PRIMARY KEY (slot_id),
    INDEX idx_shelf_pose (pose_x, pose_y)
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4;

-- ---------------------------------------------------------------------------
-- items: catalog entries
-- ---------------------------------------------------------------------------
CREATE TABLE IF NOT EXISTS items (
    item_id         INT UNSIGNED    NOT NULL AUTO_INCREMENT,
    name            VARCHAR(128)    NOT NULL,
    category_id     INT UNSIGNED    NOT NULL,
    home_slot_id    INT UNSIGNED    NOT NULL,
    weight_kg       DECIMAL(6,3)    NULL COMMENT 'retained for future use; not read at runtime',
    created_at      TIMESTAMP       NOT NULL DEFAULT CURRENT_TIMESTAMP,
    PRIMARY KEY (item_id),
    CONSTRAINT fk_items_category FOREIGN KEY (category_id) REFERENCES categories (category_id),
    CONSTRAINT fk_items_home_slot FOREIGN KEY (home_slot_id) REFERENCES shelf_slots (slot_id)
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4;

ALTER TABLE shelf_slots
    ADD CONSTRAINT fk_slots_occupied_item FOREIGN KEY (occupied_item_id) REFERENCES items (item_id);

-- ---------------------------------------------------------------------------
-- tasks: fulfilment requests (item -> drop-off pose)
-- ---------------------------------------------------------------------------
CREATE TABLE IF NOT EXISTS tasks (
    task_id         INT UNSIGNED    NOT NULL AUTO_INCREMENT,
    item_id         INT UNSIGNED    NOT NULL,
    dropoff_x       DECIMAL(8,3)    NOT NULL COMMENT 'metres, warehouse frame',
    dropoff_y       DECIMAL(8,3)    NOT NULL COMMENT 'metres, warehouse frame',
    dropoff_theta   DECIMAL(6,4)    NOT NULL COMMENT 'radians',
    priority        TINYINT UNSIGNED NOT NULL DEFAULT 5,
    status          ENUM('pending','in_progress','completed','failed') NOT NULL DEFAULT 'pending',
    created_at      TIMESTAMP       NOT NULL DEFAULT CURRENT_TIMESTAMP,
    PRIMARY KEY (task_id),
    CONSTRAINT fk_tasks_item FOREIGN KEY (item_id) REFERENCES items (item_id),
    INDEX idx_tasks_status (status)
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4;

-- ---------------------------------------------------------------------------
-- runs: one row per simulated episode attempt at a task
-- ---------------------------------------------------------------------------
CREATE TABLE IF NOT EXISTS runs (
    run_id                  INT UNSIGNED   NOT NULL AUTO_INCREMENT,
    task_id                 INT UNSIGNED   NOT NULL,
    scenario_id             VARCHAR(16)    NOT NULL COMMENT 'e.g. S-02, per docs/test-plan.md',
    robot_namespace         VARCHAR(16)    NOT NULL DEFAULT 'robot1',
    outcome                 ENUM('success','fail','aborted') NOT NULL,
    collisions_count        INT UNSIGNED   NOT NULL DEFAULT 0,
    min_clearance_m         DECIMAL(6,3)   NULL COMMENT 'metres',
    path_length_m           DECIMAL(8,3)   NULL COMMENT 'metres',
    straight_line_baseline_m DECIMAL(8,3)  NULL COMMENT 'metres',
    path_length_ratio       DECIMAL(6,3)   NULL COMMENT 'path_length_m / straight_line_baseline_m',
    time_to_goal_s          DECIMAL(8,3)   NULL COMMENT 'seconds',
    replan_count            INT UNSIGNED   NOT NULL DEFAULT 0,
    mean_linear_vel_mps     DECIMAL(5,3)   NULL COMMENT 'm/s, grouped meaning is per-run here; per-category rollups computed in the CSV exporter',
    start_time              TIMESTAMP      NOT NULL,
    end_time                TIMESTAMP      NULL,
    PRIMARY KEY (run_id),
    CONSTRAINT fk_runs_task FOREIGN KEY (task_id) REFERENCES tasks (task_id),
    INDEX idx_runs_task (task_id),
    INDEX idx_runs_scenario (scenario_id)
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4;

-- ---------------------------------------------------------------------------
-- run_events: fine-grained events within a run
-- ---------------------------------------------------------------------------
CREATE TABLE IF NOT EXISTS run_events (
    event_id        BIGINT UNSIGNED NOT NULL AUTO_INCREMENT,
    run_id          INT UNSIGNED    NOT NULL,
    event_type      ENUM('collision','replan','clearance_sample','goal_outcome') NOT NULL,
    event_time      TIMESTAMP(3)    NOT NULL DEFAULT CURRENT_TIMESTAMP(3),
    payload_json    JSON            NULL COMMENT 'type-specific detail, e.g. clearance value, contact point, replan reason, outcome detail',
    PRIMARY KEY (event_id),
    CONSTRAINT fk_events_run FOREIGN KEY (run_id) REFERENCES runs (run_id) ON DELETE CASCADE,
    INDEX idx_events_run (run_id),
    INDEX idx_events_type (event_type)
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4;

SET FOREIGN_KEY_CHECKS = 1;
