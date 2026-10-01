-- UC6 Warehouse Robot: MySQL 8 schema
-- Loaded automatically by docker compose (/docker-entrypoint-initdb.d/).
-- Only the ROS node `task_manager` reads/writes this database at runtime.
-- See docs/data-model.md for the rules behind each table.

SET NAMES utf8mb4;

-- ---------------------------------------------------------------------------
-- Catalog
-- ---------------------------------------------------------------------------

-- Category names only. Speed / margin values live in
-- ros/src/warehouse_bringup/config/motion_profiles.yaml (single source of truth).
CREATE TABLE categories (
    category_id  TINYINT UNSIGNED NOT NULL,
    name         VARCHAR(16)      NOT NULL,
    PRIMARY KEY (category_id),
    UNIQUE KEY uq_categories_name (name)
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4;

INSERT INTO categories (category_id, name) VALUES
    (1, 'Standard'),
    (2, 'Heavy'),
    (3, 'Fragile');

-- Known shelf poses in the map frame (no perception; ADR-002).
CREATE TABLE shelf_slots (
    slot_id      INT UNSIGNED     NOT NULL AUTO_INCREMENT,
    label        VARCHAR(16)      NOT NULL COMMENT 'e.g. A-03',
    pose_x       DECIMAL(8,3)     NOT NULL COMMENT 'metres, map frame',
    pose_y       DECIMAL(8,3)     NOT NULL COMMENT 'metres, map frame',
    pose_theta   DECIMAL(6,4)     NOT NULL COMMENT 'radians',
    PRIMARY KEY (slot_id),
    UNIQUE KEY uq_slots_label (label)
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4;

CREATE TABLE items (
    item_id      INT UNSIGNED     NOT NULL AUTO_INCREMENT,
    name         VARCHAR(64)      NOT NULL,
    category_id  TINYINT UNSIGNED NOT NULL,
    slot_id      INT UNSIGNED     NOT NULL,
    PRIMARY KEY (item_id),
    UNIQUE KEY uq_items_slot (slot_id),
    CONSTRAINT fk_items_category FOREIGN KEY (category_id) REFERENCES categories (category_id),
    CONSTRAINT fk_items_slot     FOREIGN KEY (slot_id)     REFERENCES shelf_slots (slot_id)
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4;

-- ---------------------------------------------------------------------------
-- Work
-- ---------------------------------------------------------------------------

CREATE TABLE tasks (
    task_id        INT UNSIGNED   NOT NULL AUTO_INCREMENT,
    item_id        INT UNSIGNED   NOT NULL,
    scenario_id    VARCHAR(8)     NOT NULL COMMENT 'S-00 .. S-05, see docs/test-plan.md',
    dropoff_x      DECIMAL(8,3)   NOT NULL,
    dropoff_y      DECIMAL(8,3)   NOT NULL,
    dropoff_theta  DECIMAL(6,4)   NOT NULL,
    status         ENUM('pending','in_progress','done') NOT NULL DEFAULT 'pending',
    created_at     TIMESTAMP      NOT NULL DEFAULT CURRENT_TIMESTAMP,
    PRIMARY KEY (task_id),
    CONSTRAINT fk_tasks_item FOREIGN KEY (item_id) REFERENCES items (item_id),
    INDEX idx_tasks_pick (scenario_id, status, task_id)
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4;

-- ---------------------------------------------------------------------------
-- Evidence
-- ---------------------------------------------------------------------------

CREATE TABLE runs (
    run_id                  INT UNSIGNED  NOT NULL AUTO_INCREMENT,
    task_id                 INT UNSIGNED  NOT NULL,
    scenario_id             VARCHAR(8)    NOT NULL,
    local_planner           ENUM('dwa','teb') NOT NULL,
    outcome                 ENUM('success','fail') NOT NULL,
    fail_reason             ENUM('nav_aborted','timeout','item_error','bridge_lost') NULL,
    start_x                 DECIMAL(8,3)  NOT NULL,
    start_y                 DECIMAL(8,3)  NOT NULL,
    collisions              INT UNSIGNED  NOT NULL DEFAULT 0,
    replans                 INT UNSIGNED  NOT NULL DEFAULT 0,
    recoveries              INT UNSIGNED  NOT NULL DEFAULT 0,
    min_clearance_m         DECIMAL(6,3)  NULL,
    path_length_m           DECIMAL(8,3)  NULL,
    baseline_m              DECIMAL(8,3)  NULL COMMENT '|start->shelf| + |shelf->dropoff|',
    path_ratio              DECIMAL(6,3)  NULL,
    duration_s              DECIMAL(8,3)  NULL COMMENT 'sim time, dispatch -> release/failure',
    dropoff_mean_speed_mps  DECIMAL(5,3)  NULL COMMENT 'drop-off leg only',
    started_at              TIMESTAMP(3)  NOT NULL,
    ended_at                TIMESTAMP(3)  NOT NULL,
    PRIMARY KEY (run_id),
    CONSTRAINT fk_runs_task FOREIGN KEY (task_id) REFERENCES tasks (task_id),
    INDEX idx_runs_scenario (scenario_id, local_planner),
    CONSTRAINT chk_fail_reason CHECK (
        (outcome = 'success' AND fail_reason IS NULL) OR
        (outcome = 'fail'    AND fail_reason IS NOT NULL)
    )
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4;

CREATE TABLE run_events (
    event_id    BIGINT UNSIGNED NOT NULL AUTO_INCREMENT,
    run_id      INT UNSIGNED    NOT NULL,
    event_type  ENUM('collision','replan','recovery','warning') NOT NULL,
    sim_time_s  DECIMAL(10,3)   NOT NULL COMMENT 'seconds since run start',
    payload     JSON            NULL COMMENT 'e.g. {"other_tag":"NPC"} or {"msg":"profile fallback"}',
    PRIMARY KEY (event_id),
    CONSTRAINT fk_events_run FOREIGN KEY (run_id) REFERENCES runs (run_id) ON DELETE CASCADE,
    INDEX idx_events_run_type (run_id, event_type)
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4;
