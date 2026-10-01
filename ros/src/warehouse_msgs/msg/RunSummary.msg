# Everything measured for one episode. Maps 1:1 to a row in the runs table
# plus its run_events. Float metrics that were not measured are NaN (stored as NULL).

string OUTCOME_SUCCESS=success
string OUTCOME_FAIL=fail

string FAIL_NAV_ABORTED=nav_aborted
string FAIL_TIMEOUT=timeout
string FAIL_ITEM_ERROR=item_error
string FAIL_BRIDGE_LOST=bridge_lost

int32 task_id
string scenario_id                 # S-00 .. S-05
string local_planner               # dwa | teb
string outcome                     # OUTCOME_*
string fail_reason                 # FAIL_* when outcome = fail, "" on success

float64 start_x                    # robot start pose, map frame
float64 start_y

uint32 collisions
uint32 replans
uint32 recoveries
float64 min_clearance_m
float64 path_length_m
float64 baseline_m                 # |start->shelf| + |shelf->dropoff|
float64 path_ratio                 # path_length_m / baseline_m
float64 duration_s                 # sim time, dispatch -> release or failure
float64 dropoff_mean_speed_mps     # drop-off leg only

time started_at
time ended_at

RunEvent[] events
